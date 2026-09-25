#include "ApplicationConfig.hpp"
// need to be included first
#include "Regulator.hpp"
#include "Remote.hpp"
#include "Valve.hpp"
#include "displayManager.hpp"
#include "kvasir/Util/BootLog.hpp"
#include "uc_log/uc_log.hpp"

namespace {
Valve::Time sinceBoot(Clock::time_point t) {
    return std::chrono::duration_cast<Valve::Time>(t.time_since_epoch());
}

template<typename Event>
Valve::ButtonEvent buttonEvent(Event type) {
    switch(type) {
    case Event::hit:          return Valve::ButtonEvent::hit;
    case Event::releaseShort: return Valve::ButtonEvent::releaseShort;
    case Event::longPress:    return Valve::ButtonEvent::longPress;
    case Event::releaseLong:  return Valve::ButtonEvent::releaseLong;
    }
    return Valve::ButtonEvent::releaseLong;
}

/// The reading for the display: a low pass (1.6 s), and the digits only move once the value
/// is 0.08 K away from what they show - a reading between two tenths does not flicker. A
/// jump of more than 3 K (a draw starts) is taken at once.
class DisplayReading {
public:
    /// Empty without a reading.
    std::optional<Units::Shown> operator()(std::optional<Units::Temperature> reading,
                                           Valve::Time                       now) {
        if(!reading) {
            filtered_.reset();
            return std::nullopt;
        }
        Units::Kelvins const jump
          = filtered_ ? Units::Kelvins{*reading - *filtered_} : Units::Kelvins{};
        if(!filtered_ || Units::magnitude(jump) > Jump) {
            filtered_ = *reading;
            at_       = now;
            shown_    = Units::shown(*reading);
        }
        while(now - at_ >= Step) {
            at_ += Step;
            *filtered_ += (*reading - *filtered_) / 16;
        }
        Units::Kelvins const off = *filtered_ - shown_;
        if(Units::magnitude(off) >= Hysteresis) { shown_ = Units::shown(*filtered_); }
        return shown_;
    }

private:
    static constexpr Valve::Time    Step{100};
    static constexpr Units::Kelvins Jump       = Units::kelvins(3'000);
    static constexpr Units::Kelvins Hysteresis = Units::kelvins(80);

    std::optional<Units::Temperature> filtered_{};
    Valve::Time                       at_{};
    Units::Shown                      shown_{};
};

/// What the regulator did, in words: the log is how it is judged from far away. Every value
/// is a quantity and brings its unit.
void log(Regulator::Event const& e) {
    using Kind = Regulator::Event::Kind;
    switch(e.kind) {
    case Kind::state:
        UC_LOG_I("regulator: {} (was {})",
                 static_cast<Regulator::State>(e.n),
                 static_cast<Regulator::State>(e.m));
        break;
    case Kind::flowStart:
        if(e.n != 0) {
            UC_LOG_I("regulator: 🚿 water flows: the loop is warming ({}), usually the pump",
                     e.rate);
        } else {
            UC_LOG_I("regulator: 🚿 water flows (reading moves {})", e.rate);
        }
        break;
    case Kind::flowEnd:
        UC_LOG_I("regulator: 💤 no sign of flow for {}: taken to stand", e.since);
        break;
    case Kind::run: UC_LOG_I("regulator: run {} -> {}, error {}", e.from, e.to, e.kelvins); break;
    case Kind::fastRun:
        UC_LOG_W("regulator: 🔥 FAST run {} -> {}, error {}", e.from, e.to, e.kelvins);
        break;
    case Kind::probe:
        UC_LOG_I("regulator: 🔍 probe {} -> {} (steady, too cold, no sign of flow), error {}",
                 e.from,
                 e.to,
                 e.kelvins);
        break;
    case Kind::fastStop:
        UC_LOG_I("regulator: fast run stopped at {}, error down to {}", e.from, e.kelvins);
        break;
    case Kind::response:
        UC_LOG_I("regulator: the reading answered with {}, position {}", e.kelvins, e.from);
        break;
    case Kind::noResponse:
        if(e.m != 0) {
            UC_LOG_W(
              "regulator: 🤷 no answer to the run ({}), {} in a row - the flow lapsed "
              "while it was waited for",
              e.kelvins,
              e.n);
        } else {
            UC_LOG_W("regulator: 🤷 no answer to the run ({}), {} in a row", e.kelvins, e.n);
        }
        break;
    case Kind::kept:
        UC_LOG_I("regulator: 📌 the water stopped while the run was judged: keeping {}", e.from);
        break;
    case Kind::takeBack:
        UC_LOG_I("regulator: ↩️ taking the runs towards hot back, {} -> {}", e.from, e.to);
        break;
    case Kind::noFlow:
        UC_LOG_W("regulator: 🚱 {} runs without an answer: no water flows", e.n);
        break;
    case Kind::limit:
        UC_LOG_W("regulator: 🛑 at the end of the travel ({}) and still {} off", e.from, e.kelvins);
        break;
    case Kind::parameters: UC_LOG_I("regulator: parameters changed"); break;
    case Kind::learnedGain:
        UC_LOG_I("regulator: 🎓 learned, region {} of 8: this run {}, now {}",
                 e.n,
                 e.measured,
                 e.gain);
        break;
    case Kind::learnRejected:
        UC_LOG_W(
          "regulator: 🎓✘ region {} of 8: this run says {}, too far from {} to be the "
          "valve - not learned",
          e.n,
          e.measured,
          e.gain);
        break;
    case Kind::learnedBacklash:
        UC_LOG_I("regulator: 🎓 learned, backlash: this run {}, now {}", e.from, e.to);
        break;
    case Kind::wrongWay:
        UC_LOG_W("regulator: 🙃 the reading answered the WRONG way ({}), {} in a row",
                 e.kelvins,
                 e.n);
        break;
    case Kind::direction:
        UC_LOG_E("regulator: 🔄 the valve acts the other way round: coldAtOpen is {} now", e.n);
        break;
    }
}
}   // namespace

/// The wiring: the buttons, the limit inputs and the clock into Valve::Controller, and what
/// it decides onto the relays, the display and the flash. What the valve does is decided in
/// Valve.hpp.
int main() {
    Kvasir::Boot::logBoot(Kvasir::PM::reset_cause());

    HW::Watchdog::init();

    Rtd                               max31865;
    DisplayManager<DisplayI2C, Clock> display{};
    Valve::Controller                 valve{CalibrationEeprom::value(),
                                            PositionJournal::read(),
                                            TargetEeprom::value()};
    Regulator::Controller             regulator{ParameterEeprom::value(), LearnedEeprom::value()};
    // the remote control: duplex channel 0 ("control") of the log backend, see Remote.hpp
    Remote<HW::ComBackend::DuplexChannel<0>> remote{};

    if(valve.calibrated()) {
        UC_LOG_I("travel: open {} close {}, position from flash {}",
                 valve.calibration().open,
                 valve.calibration().close,
                 valve.percent());
    } else {
        UC_LOG_W("no calibration in flash: measuring the travel now, any button stops it");
    }

    UC_LOG_I("target from flash: {} (0: off)", valve.target());
    // travel per kelvin by region of the travel, with the clean runs behind each
    for(std::size_t i = 0; i < Regulator::Bins; ++i) {
        if(regulator.learned().samples[i] != 0) {
            UC_LOG_I("learned from flash: region {} of 8: {} from {} runs",
                     i,
                     regulator.learned().gain[i],
                     regulator.learned().samples[i]);
        }
    }
    UC_LOG_I("learned from flash: backlash {} from {} runs",
             regulator.learned().backlash,
             regulator.learned().backlashSamples);

    DisplayReading displayReading{};

    std::array<Valve::Time, 2> lastButtonEvent{};

    auto lastMode    = valve.mode();
    auto lastOutputs = valve.outputs();
    auto nextStatus  = Valve::Time{};
    // the parameters in use, a line every 50 ms so that the log channel keeps up (with a
    // line a turn the boot lines and the last parameters were dropped)
    std::size_t parameterLogged{};
    auto        nextParameterLog = Valve::Time{500};

    while(true) {
        auto const now = sinceBoot(Clock::now());

        // Every event with the time since that button's last one: what the Controller gets.
        auto const onButton = [&](Valve::Button which, Valve::ButtonEvent event, Valve::Time at) {
            auto& last = lastButtonEvent[std::to_underlying(which)];
            UC_LOG_D("button {} {} at {}ms, {}ms after its last event",
                     which,
                     event,
                     at.count(),
                     (at - last).count());
            last = at;
            valve.button(which, event, at);
        };
        RedButton::handler([&](auto event, auto time) {
            onButton(Valve::Button::red, buttonEvent(event), sinceBoot(time));
        });
        GreenButton::handler([&](auto event, auto time) {
            onButton(Valve::Button::green, buttonEvent(event), sinceBoot(time));
        });

        auto const limits = apply(read(HW::Pin::ac1{}, HW::Pin::ac2{}));
        remote.handler(valve,
                       regulator,
                       {.open = get<0>(limits), .close = get<1>(limits)},
                       max31865,
                       now);

        // The regulator looks at the reading and asks the valve for a run; the valve takes it
        // from idle only, so the buttons, calibration and a fault come first.
        auto const regulation = regulator.tick({.reading = max31865.temperature().transform(
                                                  [](auto t) { return Units::Temperature{t}; }),
                                                .target     = valve.target(),
                                                .position   = valve.position(),
                                                .mode       = valve.mode(),
                                                .editing    = valve.editing(),
                                                .calibrated = valve.calibrated()},
                                               now);
        for(std::size_t i = 0; i < regulation.events.count; ++i) { log(regulation.events.list[i]); }
        switch(regulation.command.kind) {
        case Regulator::Command::Kind::none: break;
        case Regulator::Command::Kind::goTo:
            if(!valve.goTo(regulation.command.position)) {
                UC_LOG_W("regulator: the valve did not take the run");
            }
            break;
        case Regulator::Command::Kind::stop: valve.stop(now); break;
        case Regulator::Command::Kind::run:
            UC_LOG_W("regulator: position not known, to the cold limit");
            static_cast<void>(valve.run(regulation.command.direction));
            break;
        }
        if(regulation.saveLearned) {
            UC_LOG_I("what was learned to flash");
            LearnedEeprom::value() = regulator.learned();
            LearnedEeprom::writeValue();
        }
        if(regulation.saveParameters) {
            UC_LOG_I("parameters to flash");
            ParameterEeprom::value() = regulator.stored();
            ParameterEeprom::writeValue();
        }
        for(std::size_t i = 0; i < regulation.events.count; ++i) {
            if(regulation.events.list[i].kind == Regulator::Event::Kind::parameters) {
                parameterLogged = 0;
            }
        }
        if(parameterLogged < Regulator::ParameterCount && now >= nextParameterLog) {
            nextParameterLog = now + Valve::Time{50};
            UC_LOG_I("parameter {} {} = {} {}",
                     parameterLogged,
                     Regulator::Info[parameterLogged].name,
                     regulator.values()[parameterLogged],
                     Regulator::Info[parameterLogged].unit);
            ++parameterLogged;
        }

        auto const actions = valve.tick({.open = get<0>(limits), .close = get<1>(limits)}, now);

        // Direction first on the way in, power first on the way out: the Controller never
        // changes both in one turn, this keeps it so on the pins.
        auto const outputs = valve.outputs();
        if(outputs != lastOutputs) {
            auto const direction = [&] {
                if(outputs.direction) {
                    apply(set(HW::Pin::relay2{}));
                } else {
                    apply(clear(HW::Pin::relay2{}));
                }
            };
            if(outputs.power) {
                direction();
                apply(set(HW::Pin::relay1{}));
            } else {
                apply(clear(HW::Pin::relay1{}));
                direction();
            }
            lastOutputs = outputs;
        }

        if(actions.save) {
            UC_LOG_I("travel to flash: open {} close {}",
                     valve.calibration().open,
                     valve.calibration().close);
            CalibrationEeprom::value() = valve.calibration();
            CalibrationEeprom::writeValue();
        }
        if(actions.saveTarget) {
            UC_LOG_I("target to flash: {}", valve.target());
            TargetEeprom::value() = valve.target();
            TargetEeprom::writeValue();
        }
        if(actions.savePosition) {
            if(auto const position = valve.position()) { PositionJournal::write(*position); }
        }

        // Once a second, as metrics: uc_log_printer plots them (Metrics tab) and streams them
        // on its control socket (the metrics stream of control.sock), so a run can be recorded
        // without parsing log text.
        // -1: position not known / no reading.
        if(now >= nextStatus) {
            using namespace sc::literals;
            nextStatus             = now + std::chrono::seconds{1};
            auto const temperature = max31865.temperature();
            // Every metric with a unit is an mp-units quantity: uc_log::metric takes the unit
            // from its type. A value that is not known is not logged, instead of as -1.
            namespace mpu = mp_units;
            namespace si  = mp_units::si;
            UC_LOG_D("status: rtd faults {} target {}",
                     uc_log::metric<"rtd_faults"_sc, ""_sc, "valve"_sc>(max31865.faults()),
                     uc_log::metric<"target"_sc, "valve"_sc>(valve.target()));
            if(auto const percent = valve.percent()) {
                UC_LOG_D("status: position {}",
                         uc_log::metric<"position"_sc, "valve"_sc>(*percent));
            }
            if(temperature) {
                UC_LOG_D("status: temperature {}",
                         uc_log::metric<"temperature"_sc, "valve"_sc>(*temperature));
            }
            // a line of its own: one that gets too long is dropped whole by the printer
            auto const status = regulator.status();
            UC_LOG_D(
              "regulator: {} error {} flow {} slope {} slow {} runs {} unanswered {}",
              status.state,
              uc_log::metric<"error"_sc, "regulator"_sc>(status.error),
              uc_log::metric<"flow"_sc, ""_sc, "regulator"_sc>(status.flowing ? 1 : 0),
              uc_log::metric<"slope"_sc, "regulator"_sc>(status.fastSlope),
              uc_log::metric<"slow_slope"_sc, "regulator"_sc>(status.slowSlope),
              uc_log::metric<"runs"_sc, ""_sc, "regulator"_sc>(
                static_cast<std::int32_t>(status.runs)),
              uc_log::metric<"unanswered"_sc, ""_sc, "regulator"_sc>(int{status.noResponses}));
        }

        if(valve.mode() != lastMode) {
            lastMode = valve.mode();
            UC_LOG_D("mode {} fault {} position {}", lastMode, valve.fault(), valve.percent());
        }

        auto const shownReading = displayReading(max31865.temperature(), now);
        auto const view         = valve.view(now);
        switch(view.kind) {
        case Valve::View::Kind::temperature:
        case Valve::View::Kind::moving:      display.showTemperature(shownReading); break;
        case Valve::View::Kind::position:    display.showPercent(view.percent); break;
        case Valve::View::Kind::calibrating: display.showText("CAL"); break;
        case Valve::View::Kind::fault:       display.showText("Err"); break;
        case Valve::View::Kind::manual:      display.showText("HAn"); break;
        case Valve::View::Kind::target:
            if(view.target == Valve::Off) {
                display.showText("OFF");
            } else {
                display.showTemperature(view.target);
            }
            break;
        }
        display.blink(view.blink);

        display.handler();
        max31865.handler();
        StackProtector::handler();
        HW::Watchdog::handler();
    }
}

KVASIR_START(Startup)
