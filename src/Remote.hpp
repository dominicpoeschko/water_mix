#pragma once
#include "Protocol.hpp"
//

#include <array>
#include <cstddef>
#include <kvasir/Util/StaticVector.hpp>
#include <span>
#include <uc_log/uc_log.hpp>
#include <variant>

/// The valve's end of the remote control: Protocol.hpp over duplex channel `Channel` of the log
/// backend, which uc_log_printer bridges to a socket (duplex.0.sock). A command does what a button on the
/// board does - it goes into Valve::Controller as button events - or sets one of the
/// regulator's parameters; the State and the regulator's Status go out whenever they have
/// changed.
template<typename Channel>
struct Remote {
    template<typename Rtd>
    void handler(Valve::Controller&     valve,
                 Regulator::Controller& regulator,
                 Valve::Limits const&   limits,
                 Rtd const&             rtd,
                 Valve::Time            now) {
        receive(valve, regulator, now);

        // what could not be written last time goes first: the up buffer takes what fits
        if(!flush()) { return; }

        // The position moves with every turn of the loop while the motor runs, the reading
        // with every conversion: on their own they are worth a State every SlowPeriod.
        auto const current = state(valve, limits, rtd, now);
        auto       steady  = current;
        steady.position    = sent_.position;
        steady.temperature = sent_.temperature;
        steady.rtdFaults   = sent_.rtdFaults;
        if(stateWanted_ || steady != sent_ || (current != sent_ && now - sentAt_ > SlowPeriod)) {
            stateWanted_ = false;
            sent_        = current;
            sentAt_      = now;
            send(Protocol::Report{current});
            return;
        }

        // one frame a turn: the regulator's numbers like the position above
        auto const regulation = Protocol::Regulation{regulator.status()};
        auto       slow       = regulation;
        slow.status.reading   = sentRegulation_.status.reading;
        slow.status.error     = sentRegulation_.status.error;
        slow.status.fastSlope = sentRegulation_.status.fastSlope;
        slow.status.slowSlope = sentRegulation_.status.slowSlope;
        if(regulationWanted_ || slow != sentRegulation_
           || (regulation != sentRegulation_ && now - sentRegulationAt_ > SlowPeriod))
        {
            regulationWanted_ = false;
            sentRegulation_   = regulation;
            sentRegulationAt_ = now;
            send(Protocol::Report{regulation});
            return;
        }

        if(parametersWanted_) {
            parametersWanted_ = false;
            send(Protocol::Report{Protocol::Parameters{regulator.values()}});
            return;
        }

        if(learnedWanted_ || regulator.learned() != sentLearned_) {
            learnedWanted_ = false;
            sentLearned_   = regulator.learned();
            send(Protocol::Report{Protocol::Learned{sentLearned_}});
        }
    }

private:
    static constexpr Valve::Time SlowPeriod{250};

    void receive(Valve::Controller&     valve,
                 Regulator::Controller& regulator,
                 Valve::Time            now) {
        std::array<std::byte, 32> chunk{};
        auto const                got = Channel::read(chunk);
        for(auto const byte : got) {
            if(in_.size() == in_.max_size()) { in_.clear(); }
            in_.push_back(byte);
        }
        while(!in_.empty()) {
            Protocol::Command command{};
            auto const        result = Protocol::Packager::unpack(in_, command);
            auto const        used   = result ? result->consumed : result.error().consumed;
            in_.erase(in_.begin(), std::next(in_.begin(), static_cast<std::ptrdiff_t>(used)));
            if(!result) {
                if(used == 0) { break; }
                continue;
            }
            std::visit([&](auto const& c) { run(valve, regulator, now, c); }, command);
        }
    }

    /// A press as long as the shortest one the Controller takes for a press.
    void run(Valve::Controller& valve,
             Regulator::Controller&,
             Valve::Time            now,
             Protocol::Press const& press) {
        UC_LOG_I("remote: press {}", press.button);
        valve.button(press.button, Valve::ButtonEvent::hit, now - Valve::Controller::MinPress);
        valve.button(press.button, Valve::ButtonEvent::releaseShort, now);
    }

    /// The long press comes LongPress after the hit on the board; the Controller only looks
    /// at which button it was.
    void run(Valve::Controller& valve,
             Regulator::Controller&,
             Valve::Time           now,
             Protocol::Hold const& hold) {
        UC_LOG_I("remote: hold {}", hold.button);
        valve.button(hold.button, Valve::ButtonEvent::hit, now);
        valve.button(hold.button, Valve::ButtonEvent::longPress, now);
        valve.button(hold.button, Valve::ButtonEvent::releaseLong, now);
    }

    void run(Valve::Controller& valve,
             Regulator::Controller&,
             Valve::Time,
             Protocol::SetTarget const& target) {
        UC_LOG_I("remote: target {}", target.target);
        valve.setTarget(target.target);
    }

    void run(Valve::Controller& valve,
             Regulator::Controller&,
             Valve::Time now,
             Protocol::Calibrate const&) {
        UC_LOG_I("remote: calibrate");
        valve.button(Valve::Button::green, Valve::ButtonEvent::hit, now);
        valve.button(Valve::Button::red, Valve::ButtonEvent::hit, now);
        valve.button(Valve::Button::green, Valve::ButtonEvent::longPress, now);
        valve.button(Valve::Button::green, Valve::ButtonEvent::releaseLong, now);
        valve.button(Valve::Button::red, Valve::ButtonEvent::releaseLong, now);
    }

    void run(Valve::Controller&,
             Regulator::Controller&,
             Valve::Time,
             Protocol::Get const&) {
        stateWanted_      = true;
        regulationWanted_ = true;
    }

    void run(Valve::Controller&,
             Regulator::Controller&        regulator,
             Valve::Time                   now,
             Protocol::SetParameter const& parameter) {
        if(parameter.index < Regulator::ParameterCount) {
            UC_LOG_I("remote: parameter {} = {}",
                     Regulator::Info[parameter.index].name,
                     parameter.value);
        }
        static_cast<void>(regulator.set(parameter.index, parameter.value, now));
        parametersWanted_ = true;
    }

    void run(Valve::Controller&,
             Regulator::Controller& regulator,
             Valve::Time            now,
             Protocol::ResetParameters const&) {
        UC_LOG_I("remote: parameters to their defaults");
        regulator.resetParameters(now);
        parametersWanted_ = true;
    }

    void run(Valve::Controller&,
             Regulator::Controller&,
             Valve::Time,
             Protocol::GetParameters const&) {
        parametersWanted_ = true;
        learnedWanted_    = true;
    }

    void run(Valve::Controller& valve,
             Regulator::Controller&,
             Valve::Time,
             Protocol::GoTo const& to) {
        bool const taken = valve.goTo(to.position);
        if(taken) {
            UC_LOG_I("remote: go to {}", to.position);
        } else {
            UC_LOG_W("remote: go to {} NOT taken: not idle, or position not known", to.position);
        }
    }

    void run(Valve::Controller& valve,
             Regulator::Controller&,
             Valve::Time now,
             Protocol::Stop const&) {
        UC_LOG_I("remote: stop");
        valve.stop(now);
        if(valve.mode() == Valve::Mode::calibrating) {
            // what a button on the board does to a calibration
            valve.button(Valve::Button::green,
                         Valve::ButtonEvent::hit,
                         now - Valve::Controller::MinPress);
            valve.button(Valve::Button::green, Valve::ButtonEvent::releaseShort, now);
        }
    }

    void run(Valve::Controller&,
             Regulator::Controller& regulator,
             Valve::Time            now,
             Protocol::ResetLearned const&) {
        UC_LOG_I("remote: forget what was learned");
        regulator.resetLearned(now);
    }

    template<typename Rtd>
    [[nodiscard]] static Protocol::State state(Valve::Controller const& valve,
                                               Valve::Limits const&     limits,
                                               Rtd const&               rtd,
                                               Valve::Time              now) {
        auto const outputs     = valve.outputs();
        auto const view        = valve.view(now);
        auto const temperature = rtd.temperature();
        return {.mode        = valve.mode(),
                .fault       = valve.fault(),
                .power       = outputs.power,
                .direction   = outputs.direction,
                .limitOpen   = limits.open,
                .limitClose  = limits.close,
                .calibrated  = valve.calibrated(),
                .calibration = valve.calibration(),
                .position    = Protocol::Maybe<Valve::Position>::of(valve.position()),
                .temperature = Protocol::Maybe<Units::Temperature>::of(
                  temperature.transform([](auto t) { return Units::Temperature{t}; })),
                .rtdFault    = rtd.fault(),
                .rtdFaults   = rtd.faults(),
                .view        = view.kind,
                .blink       = view.blink,
                .target      = valve.target(),
                .shownTarget = view.target};
    }

    void send(Protocol::Report const& report) {
        out_.clear();
        outSent_ = 0;
        if(!Protocol::Packager::pack(out_, report)) { out_.clear(); }
        static_cast<void>(flush());
    }

    /// True when nothing is waiting to go out any more.
    [[nodiscard]] bool flush() {
        auto const pending = std::span<std::byte const>{out_}.subspan(outSent_);
        auto const rest    = Channel::write(pending);
        // nobody listening (no debugger): drop it rather than wait for ever
        if(rest.size() == pending.size() && !pending.empty() && ++stalled_ > 10'000U) {
            out_.clear();
            outSent_ = 0;
            stalled_ = 0;
        }
        if(rest.size() != pending.size()) { stalled_ = 0; }
        outSent_ += pending.size() - rest.size();
        return outSent_ == out_.size();
    }

    Kvasir::StaticVector<std::byte, 96> in_{};
    Kvasir::StaticVector<std::byte, 112>
                         out_{};   // Protocol::PackagerConfig::MaxSize and the frame
    std::size_t          outSent_{};
    std::uint32_t        stalled_{};
    bool                 stateWanted_{true};
    bool                 regulationWanted_{true};
    bool                 parametersWanted_{true};
    bool                 learnedWanted_{true};
    Regulator::Learned   sentLearned_{};
    Protocol::State      sent_{};
    Valve::Time          sentAt_{};
    Protocol::Regulation sentRegulation_{};
    Valve::Time          sentRegulationAt_{};
};
