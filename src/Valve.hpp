#pragma once

#include "Units.hpp"

#include <array>
#include <chrono>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>

/// What the valve does, free of the SDK: button events, the two limit inputs and the time
/// in; the two relays (Outputs), what the display shows (View) and a few things for main()
/// to carry out (Actions) out. main() is the wiring; everything that decides is here, and
/// tested by the static_asserts at the end.
///
/// The two buttons: the display shows the temperature; a press opens the target temperature
/// (blinking), red then moves it up and green down by TargetStep - held, they keep stepping
/// - and EditTimeout after the last press it is taken and handed to main() for the flash.
/// Below TargetMin it is "OFF". Both buttons held for ManualHold switch the motor by hand
/// on and off ("HAn": green held opens, red held closes, for as long as it is held); both
/// held on to the long press start a calibration. README.md has it for the user.
/// Holding the target temperature is Regulator.hpp's: it asks for runs with goTo() / stop(),
/// which are taken from idle only.
///
/// The motor is a three-point actuator: one relay powers it, one picks the direction. The
/// direction relay never switches under load -- it is set DirectionSettle before the power
/// comes on and released as long after it went off -- and a run in one direction is
/// ReverseDeadTime away from the last one. A run ends at its limit input, at a button, or
/// at its time limit, which is a fault: a limit that never comes is a broken switch or wire.
///
/// Calibration drives to the close limit, then times a full run open and a full run close,
/// and hands both to main() to keep in flash. It starts by itself when there is none at boot.
/// From the travel times and from the time the motor has run since a limit was last seen,
/// position() says where the valve is; main() keeps that in flash too, so it is known after
/// a power loss. Every run that ends at its limit corrects the travel time a little: the
/// limit came earlier or later than the position said.
namespace Valve {

/// Time since boot.
using Time = std::chrono::milliseconds;

enum class Direction : std::uint8_t { open, close };

enum class Button : std::uint8_t { green, red };

enum class ButtonEvent : std::uint8_t { hit, releaseShort, longPress, releaseLong };

/// The limit inputs as read, unfiltered.
struct Limits {
    bool open{};
    bool close{};
};

/// The relays.
struct Outputs {
    bool power{};
    bool direction{};   ///< set: open

    constexpr bool operator==(Outputs const&) const = default;
};

/// How long a full run takes. 32 bits of milliseconds: the flash format.
using TravelTime = std::chrono::duration<std::uint32_t, std::milli>;

using Units::Percent;
using Units::Position;

/// A full run in each direction. The layout is the flash format -- a firmware that changes
/// it reads the stored calibration as invalid. All zero: none.
struct Calibration {
    TravelTime open{};
    TravelTime close{};

    constexpr auto operator<=>(Calibration const&) const = default;
};

enum class Mode : std::uint8_t {
    idle,
    moving,
    calibrating,
    fault,
    manual,   ///< by hand: a held button runs the motor
};

enum class Fault : std::uint8_t {
    none,
    timeout,      ///< the limit did not come within the run's time limit
    bothLimits,   ///< both limit inputs at once: wiring
};

/// What main() has to do after a call into the Controller.
struct Actions {
    bool save{};           ///< calibration() is new: write it to flash
    bool savePosition{};   ///< position() moved: write it to flash
    bool saveTarget{};     ///< target() is new: write it to flash
};

/// A temperature to set, in tenths of a degree Celsius; as a target, zero is "off".
using Units::Target;
inline constexpr Target Off = Units::target(0);

/// What the display shows.
struct View {
    enum class Kind : std::uint8_t {
        temperature,   ///< the reading, steady
        position,      ///< percent open, blinking while the motor runs
        moving,        ///< the motor runs and the position is not known: the reading, blinking
        calibrating,   ///< "CAL", blinking
        fault,         ///< "Err"
        target,        ///< the target temperature being set, blinking; "OFF" for 0
        manual,        ///< "HAn": the motor by hand; blinking while both buttons are still held
    };

    Kind    kind{Kind::temperature};
    Percent percent{};   ///< for Kind::position
    Target  target{};    ///< for Kind::target
    bool    blink{};

    constexpr bool operator==(View const&) const = default;
};

class Controller {
public:
    /// The direction relay's contacts are at rest this long before the power relay closes
    /// and this long after it opened.
    static constexpr Time DirectionSettle{60};
    /// Between the end of a run and the start of the next one.
    static constexpr Time ReverseDeadTime{500};
    /// A limit input counts once it has held its level for this long. Covers contact bounce
    /// and, on an input that follows the mains through an optocoupler, the zero crossings.
    static constexpr Time LimitFilter{40};
    /// A press shorter than this is not one. The contacts are debounced before they get here
    /// (PushButton's debouncePress / debounceRelease): a light tap is 12 ms, and the 30 ms
    /// this used to be threw half of them away.
    static constexpr Time MinPress{3};
    /// A full run takes at least / at most this long; what calibration measures outside of
    /// it is not kept, and an uncalibrated run is stopped at MaxTravel.
    static constexpr Time MinTravel{1'000};
    static constexpr Time MaxTravel{300'000};
    /// A calibrated run may take this much longer than measured before it is a fault:
    /// a quarter of the travel, and a little for a short one.
    static constexpr Time TravelMargin{2'000};
    /// The position stays on the display this long after the motor stopped.
    static constexpr Time ShowPosition{3'000};
    /// While the motor runs the position goes to flash this often, and once more when it
    /// stopped: a power loss in mid-run costs this much travel at most.
    static constexpr Time PositionSavePeriod{1'000};

    /// position(): Closed (0 %) .. Full (100 %, open).
    static constexpr Position Closed = Units::Closed;
    static constexpr Position Full   = Units::Full;

    [[nodiscard]] static constexpr bool plausible(Calibration const& c) {
        auto const ok = [](TravelTime t) { return t >= MinTravel && t <= MaxTravel; };
        return ok(c.open) && ok(c.close);
    }

    /// The target temperature: off, or TargetMin..TargetMax in steps of TargetStep.
    static constexpr Target TargetMin  = Units::target(200);
    static constexpr Target TargetMax  = Units::target(650);
    static constexpr Target TargetStep = Units::target(5);
    /// The target on the display is taken this long after the last press.
    static constexpr Time EditTimeout{3'000};
    /// Both buttons held this long: the motor by hand, on or off.
    static constexpr Time ManualHold{1'000};
    /// By hand, one button held this long runs the motor: two fingers that do not come down
    /// in the same instant must not move it.
    static constexpr Time ManualStartDelay{300};
    /// By hand ends by itself this long after the last button.
    static constexpr Time ManualTimeout{60'000};
    /// A button held while the target is on the display steps it again after RepeatDelay,
    /// then every RepeatPeriod.
    static constexpr Time RepeatDelay{600};
    static constexpr Time RepeatPeriod{150};

    [[nodiscard]] static constexpr bool plausibleTarget(Target t) {
        return t == Off || (t >= TargetMin && t <= TargetMax);
    }

    /// A run that ends at its limit and came at least this far corrects the travel time; a
    /// shorter one says too little about it.
    static constexpr Position AdaptMinDistance = Full / 4;
    /// The corrected travel time goes to flash once it is this far (1/SaveChange) from what
    /// flash holds.
    static constexpr std::uint32_t SaveChange = 100;

    /// `stored`, `storedPosition` and `storedTarget` are what flash holds; times no actuator
    /// can have are not used, and a position is worth nothing without them. Without a calibration the
    /// first thing the valve does is measure one.
    constexpr explicit Controller(Calibration const&      stored         = {},
                                  std::optional<Position> storedPosition = {},
                                  Target                  storedTarget   = Off)
      : calibration_{plausible(stored) ? stored : Calibration{}}
      , saved_{calibration_}
      , target_{plausibleTarget(storedTarget) ? storedTarget : Off} {
        if(!calibrated()) {
            startCalibration_();
        } else if(storedPosition && *storedPosition >= Closed && *storedPosition <= Full) {
            position_      = *storedPosition;
            referenced_    = true;
            savedPosition_ = storedPosition;
        }
    }

    // -- in ---------------------------------------------------------------------------------------

    /// A button event. Short press: the target temperature, and once it is on the display
    /// up (red) or down (green); any short press while the motor runs or calibration is under
    /// way stops it, and acknowledges a fault. Long press while the other button is held
    /// too: calibration. A long press that comes with its hit (nothing was held: the remote
    /// control) runs the motor by hand, open (green) or close (red).
    constexpr void button(Button      which,
                          ButtonEvent event,
                          Time        now) {
        std::size_t const i = std::to_underlying(which);
        switch(event) {
        case ButtonEvent::hit:
            hitAt_[i]    = now;
            held_[i]     = true;
            repeated_[i] = false;
            break;
        case ButtonEvent::releaseShort:
            held_[i] = false;
            // a hold that stepped the target already is not a press on top of that
            // ... nor is letting go after both were held to switch by-hand
            if(now - hitAt_[i] >= MinPress && !repeated_[i] && !bothHandled_) {
                pressed_(which, now);
            }
            break;
        case ButtonEvent::longPress:
            if(held_[1 - i] && (mode_ == Mode::idle || mode_ == Mode::manual)) {
                endEdit_();
                stop_(now);
                startCalibration_();
                break;
            }
            if(mode_ != Mode::idle) { break; }
            if(!editing_) {
                mode_ = Mode::moving;
                request_(which == Button::green ? Direction::open : Direction::close);
            }
            break;
        case ButtonEvent::releaseLong: held_[i] = false; break;
        }
    }

    /// The target from somewhere else than the buttons (the remote control); what is not a
    /// target is not taken.
    constexpr void setTarget(Target t) {
        if(!plausibleTarget(t)) { return; }
        editing_ = false;
        target_  = t;
    }

    /// To `position` (0 closed .. Full open), for the regulator: from idle and with a known
    /// position only. 0 and Full are runs to the limit switch, which is what says where the
    /// valve is; anything between ends where the travel times say it is reached. A button
    /// stops it like any other run.
    constexpr bool goTo(Position position) {
        auto const from = this->position();
        if(mode_ != Mode::idle || editing_ || !from) { return false; }
        position = Units::clamp(position, Closed, Full);
        if(position == *from && position != Closed && position != Full) { return true; }
        mode_ = Mode::moving;
        if(position > Closed && position < Full) { goal_ = position; }
        request_(position > *from || position == Full ? Direction::open : Direction::close);
        return true;
    }

    /// To the limit of `d`, from idle: what the remote control's long press does.
    constexpr bool run(Direction d) {
        if(mode_ != Mode::idle || editing_) { return false; }
        mode_ = Mode::moving;
        request_(d);
        return true;
    }

    /// Ends a run that goTo() or run() started (not a calibration).
    constexpr void stop(Time now) {
        if(mode_ != Mode::moving) { return; }
        stop_(now);
        mode_ = Mode::idle;
    }

    /// Every turn of the main loop.
    constexpr Actions tick(Limits raw,
                           Time   now) {
        Actions a{};
        edit_(now);
        manual_(now);
        if(target_ != savedTarget_) {
            savedTarget_ = target_;
            a.saveTarget = true;
        }
        filter_(open_, raw.open, now);
        filter_(close_, raw.close, now);
        track_(now);

        // A limit says where the valve is, whether the motor brought it there or not.
        if(open_.level && close_.level) {
            if(mode_ != Mode::fault) { fail_(Fault::bothLimits, now); }
        } else if(open_.level || close_.level) {
            position_   = open_.level ? Full : Closed;
            referenced_ = true;
        }

        switch(phase_) {
        case Phase::off:
            if(!pending_ || now < earliestStart_) { break; }
            direction_ = *pending_;
            pending_.reset();
            if(atLimit_(direction_)) {
                arrived_(a, now, std::nullopt);
            } else {
                phase_    = Phase::settle;
                phaseEnd_ = now + DirectionSettle;
            }
            break;
        case Phase::settle:
            if(now >= phaseEnd_) {
                phase_            = Phase::run;
                runStart_         = now;
                runFrom_          = position_;
                runReferenced_    = referenced_;
                runFromLimit_     = open_.level || close_.level;
                deadline_         = now + runLimit_(direction_);
                nextPositionSave_ = now + PositionSavePeriod;
            }
            break;
        case Phase::run:
            if(atLimit_(direction_)) {
                auto const ran = now - runStart_;
                stop_(now);
                arrived_(a, now, ran);
            } else if(goal_
                      && (direction_ == Direction::open ? position_ >= *goal_
                                                        : position_ <= *goal_))
            {
                stop_(now);
                mode_ = Mode::idle;
            } else if(now >= deadline_) {
                fail_(Fault::timeout, now);
            }
            break;
        case Phase::release:
            if(now >= phaseEnd_) { phase_ = Phase::off; }
            break;
        }

        if(auto const p = position(); p && !Units::same(p, savedPosition_)
                                      && (phase_ != Phase::run || now >= nextPositionSave_))
        {
            savedPosition_    = p;
            nextPositionSave_ = now + PositionSavePeriod;
            a.savePosition    = true;
        }
        return a;
    }

    // -- out --------------------------------------------------------------------------------------

    [[nodiscard]] constexpr Outputs outputs() const {
        bool const engaged = phase_ != Phase::off;
        return {.power     = phase_ == Phase::run,
                .direction = engaged && direction_ == Direction::open};
    }

    [[nodiscard]] constexpr View view(Time now) const {
        switch(mode_) {
        case Mode::fault:       return {.kind = View::Kind::fault};
        case Mode::calibrating: return {.kind = View::Kind::calibrating, .blink = true};
        case Mode::moving:
            if(auto const p = percent()) {
                return {.kind = View::Kind::position, .percent = *p, .blink = true};
            }
            return {.kind = View::Kind::moving, .blink = true};
        case Mode::manual:
            if(auto const p = percent();
               p && (phase_ != Phase::off || (showUntil_ && now < *showUntil_)))
            {
                return {.kind = View::Kind::position, .percent = *p, .blink = phase_ != Phase::off};
            }
            return {.kind = View::Kind::manual, .blink = bothHandled_};
        case Mode::idle: break;
        }
        if(editing_) { return {.kind = View::Kind::target, .target = edited_, .blink = true}; }
        if(auto const p = percent(); p && showUntil_ && now < *showUntil_) {
            return {.kind = View::Kind::position, .percent = *p};
        }
        return {};
    }

    [[nodiscard]] constexpr Mode mode() const { return mode_; }

    /// The target temperature is on the display, being set.
    [[nodiscard]] constexpr bool editing() const { return editing_; }

    [[nodiscard]] constexpr Fault fault() const { return fault_; }

    /// The temperature the valve is to hold; 0: none.
    [[nodiscard]] constexpr Target target() const { return target_; }

    [[nodiscard]] constexpr bool calibrated() const { return calibration_ != Calibration{}; }

    [[nodiscard]] constexpr Calibration const& calibration() const { return calibration_; }

    /// 0 closed .. Full open; empty without a calibration, and until a limit has been seen
    /// since boot: the valve may have been anywhere when the power came on.
    [[nodiscard]] constexpr std::optional<Position> position() const {
        if(!calibrated() || !referenced_) { return std::nullopt; }
        return position_;
    }

    /// position(), rounded to a percent.
    [[nodiscard]] constexpr std::optional<Percent> percent() const {
        return position().transform(Units::wholePercent);
    }

private:
    enum class Phase : std::uint8_t {
        off,       ///< both relays released
        settle,    ///< direction set, waiting to power
        run,       ///< powered
        release,   ///< power off, direction still held
    };

    enum class Step : std::uint8_t { home, open, close };

    struct Filtered {
        bool level{};
        bool last{};
        Time since{};
    };

    static constexpr void filter_(Filtered& f,
                                  bool      raw,
                                  Time      now) {
        if(raw != f.last) {
            f.last  = raw;
            f.since = now;
        }
        if(f.level != raw && now - f.since >= LimitFilter) { f.level = raw; }
    }

    [[nodiscard]] constexpr bool atLimit_(Direction d) const {
        return d == Direction::open ? open_.level : close_.level;
    }

    [[nodiscard]] constexpr Time travel_(Direction d) const {
        return d == Direction::open ? calibration_.open : calibration_.close;
    }

    /// How long a run may take. While calibrating nothing is known yet.
    [[nodiscard]] constexpr Time runLimit_(Direction d) const {
        if(!calibrated() || mode_ == Mode::calibrating) { return MaxTravel; }
        return travel_(d) + travel_(d) / 4 + TravelMargin;
    }

    /// The position from where the run started and how long it has lasted: no error adds up
    /// over the turns of the loop.
    constexpr void track_(Time now) {
        if(phase_ != Phase::run || !calibrated() || !referenced_) { return; }
        auto const travel = travel_(direction_);
        auto const ran    = now - runStart_;
        // the share of the travel time that has run is the share of the travel (64 bits:
        // 100 000 times the milliseconds does not fit 32)
        auto const moved
          = ran >= travel
            ? Full
            : Units::mpu::value_cast<std::int32_t>(Units::mpu::value_cast<std::int64_t>(Full)
                                                   * ran.count() / travel.count());
        auto const p = direction_ == Direction::open ? runFrom_ + moved : runFrom_ - moved;
        position_    = Units::clamp(p, Closed, Full);
    }

    constexpr void request_(Direction d) { pending_ = d; }

    /// int16 arithmetic is done in int: back into the target's 16 bits.
    [[nodiscard]] static constexpr Target narrow_(auto sum) {
        return Units::mpu::value_cast<std::int16_t>(sum);
    }

    /// One step, red up and green down; "off" sits one step below TargetMin.
    constexpr void stepTarget_(Button which) {
        if(which == Button::red) {
            edited_ = edited_ == Off ? TargetMin : narrow_(edited_ + TargetStep);
            if(edited_ > TargetMax) { edited_ = TargetMax; }
        } else {
            edited_ = edited_ <= TargetMin ? Off : narrow_(edited_ - TargetStep);
        }
    }

    constexpr void endEdit_() {
        if(!editing_) { return; }
        editing_ = false;
        target_  = edited_;
    }

    /// The target on the display: a held button keeps stepping, and EditTimeout after the
    /// last press it is taken.
    constexpr void edit_(Time now) {
        if(mode_ != Mode::idle) {
            endEdit_();
            return;
        }
        for(std::size_t i = 0; i < held_.size(); ++i) {
            if(!held_[i] || held_[1 - i] || now - hitAt_[i] < RepeatDelay) { continue; }
            if(!editing_) {
                // held from the temperature: the target comes up, and steps from the next
                // RepeatDelay on
                editing_     = true;
                edited_      = target_;
                repeated_[i] = true;
                nextRepeat_  = now + RepeatDelay;
            } else if(now >= nextRepeat_) {
                stepTarget_(static_cast<Button>(i));
                repeated_[i] = true;
                nextRepeat_  = now + RepeatPeriod;
            }
            editUntil_ = now + EditTimeout;
        }
        if(!editing_) { return; }
        if(now >= editUntil_ && !held_[0] && !held_[1]) { endEdit_(); }
    }

    /// The motor by hand. Both buttons held for ManualHold switch it on and off (once per
    /// hold); in it, one button held runs the motor - green opens, red closes - and letting
    /// go stops it.
    constexpr void manual_(Time now) {
        bool const both = held_[0] && held_[1];
        if(!held_[0] && !held_[1]) { bothHandled_ = false; }
        if(both && !bothHandled_ && (mode_ == Mode::idle || mode_ == Mode::manual)
           && now - (hitAt_[0] > hitAt_[1] ? hitAt_[0] : hitAt_[1]) >= ManualHold)
        {
            bothHandled_ = true;
            if(mode_ == Mode::idle) {
                endEdit_();
                mode_        = Mode::manual;
                manualUntil_ = now + ManualTimeout;
            } else {
                stop_(now);
                mode_ = Mode::idle;
            }
        }
        if(mode_ != Mode::manual) { return; }

        bool const engaged = pending_ || phase_ == Phase::settle || phase_ == Phase::run;
        std::optional<Direction> wanted{};
        if(!both && !bothHandled_) {
            for(std::size_t i = 0; i < held_.size(); ++i) {
                if(held_[i] && now - hitAt_[i] >= ManualStartDelay) {
                    wanted = static_cast<Button>(i) == Button::green ? Direction::open
                                                                     : Direction::close;
                }
            }
        }
        if(held_[0] || held_[1]) { manualUntil_ = now + ManualTimeout; }
        if(wanted) {
            if(!engaged && phase_ == Phase::off) { request_(*wanted); }
        } else if(engaged) {
            stop_(now);
        }
        if(now >= manualUntil_ && phase_ == Phase::off && !pending_) { mode_ = Mode::idle; }
    }

    /// Power off now; the direction relay follows.
    constexpr void stop_(Time now) {
        pending_.reset();
        goal_.reset();
        if(phase_ == Phase::off) { return; }
        bool const powered = phase_ == Phase::run;
        phase_             = Phase::release;
        phaseEnd_          = now + DirectionSettle;
        if(powered) {
            earliestStart_ = now + ReverseDeadTime;
            showUntil_     = now + ShowPosition;
        }
    }

    constexpr void fail_(Fault f,
                         Time  now) {
        stop_(now);
        mode_  = Mode::fault;
        fault_ = f;
    }

    constexpr void startCalibration_() {
        mode_ = Mode::calibrating;
        step_ = Step::home;
        request_(Direction::close);
    }

    /// The limit of the run's direction is there. `ran` is how long the motor was powered to
    /// get to it; empty when the valve stood at the limit already.
    constexpr void arrived_(Actions&            a,
                            Time                now,
                            std::optional<Time> ran) {
        if(mode_ != Mode::calibrating) {
            if(ran) { adapt_(a, *ran); }
            if(mode_ != Mode::manual) { mode_ = Mode::idle; }
            return;
        }
        switch(step_) {
        case Step::home:
            step_ = Step::open;
            request_(Direction::open);
            break;
        case Step::open:
            // From the close limit: a run that did not happen or was too short to be a full
            // one is the limit inputs, not the valve.
            if(!ran || *ran < MinTravel) {
                fail_(Fault::timeout, now);
                break;
            }
            measured_.open = std::chrono::duration_cast<TravelTime>(*ran);
            step_          = Step::close;
            request_(Direction::close);
            break;
        case Step::close:
            if(!ran || *ran < MinTravel) {
                fail_(Fault::timeout, now);
                break;
            }
            measured_.close = std::chrono::duration_cast<TravelTime>(*ran);
            calibration_    = measured_;
            saved_          = calibration_;
            a.save          = true;
            mode_           = Mode::idle;
            break;
        }
    }

    /// A run of `ran` ended at its limit. Had the travel time been right, the run would have
    /// taken the travel's share between where it started and the limit; what it took instead
    /// says what the travel is. That counts by the share of the travel the run covered, and
    /// half of that again: one stiff run must not move it far. A start away from a limit is
    /// itself an estimate and counts half once more.
    constexpr void adapt_(Actions& a,
                          Time     ran) {
        if(!calibrated() || !runReferenced_) { return; }
        Position const distance = direction_ == Direction::open ? Full - runFrom_ : runFrom_;
        if(distance < AdaptMinDistance) { return; }
        Time const estimate{Units::share(ran.count(), Full, distance)};
        if(estimate < MinTravel || estimate > MaxTravel) { return; }

        auto&       travel = direction_ == Direction::open ? calibration_.open : calibration_.close;
        auto const& inFlash       = direction_ == Direction::open ? saved_.open : saved_.close;
        std::int64_t const weight = runFromLimit_ ? 2 : 4;
        Time const         was    = travel;
        travel                    = std::chrono::duration_cast<TravelTime>(
          was + Time{Units::share((estimate - was).count(), distance, Full) / weight});

        auto const off = travel > inFlash ? travel - inFlash : inFlash - travel;
        if(off * SaveChange >= inFlash) {
            saved_ = calibration_;
            a.save = true;
        }
    }

    constexpr void pressed_(Button which,
                            Time   now) {
        switch(mode_) {
        case Mode::fault:
            mode_  = Mode::idle;
            fault_ = Fault::none;
            break;
        case Mode::calibrating:
        case Mode::moving:
            stop_(now);
            mode_ = Mode::idle;
            break;
        case Mode::manual: break;   // held buttons run the motor: manual_()
        case Mode::idle:
            // the first press only shows the target
            if(editing_) {
                stepTarget_(which);
            } else {
                editing_ = true;
                edited_  = target_;
            }
            editUntil_ = now + EditTimeout;
            break;
        }
    }

    Calibration calibration_{};
    Calibration saved_{};   ///< what flash holds
    Calibration measured_{};

    Mode  mode_{Mode::idle};
    Fault fault_{Fault::none};
    Step  step_{Step::home};

    Phase                    phase_{Phase::off};
    Direction                direction_{Direction::close};
    std::optional<Direction> pending_{};
    std::optional<Position>  goal_{};   ///< goTo(): the run ends here
    Time                     phaseEnd_{};
    Time                     earliestStart_{};
    Time                     runStart_{};
    Time                     deadline_{};

    Position            position_{};
    Position            runFrom_{};
    bool                referenced_{false};
    bool                runReferenced_{false};   ///< the position was known when the run began
    bool                runFromLimit_{false};    ///< ... because the valve stood at a limit
    std::optional<Time> showUntil_{};

    std::optional<Position> savedPosition_{};   ///< what flash holds
    Time                    nextPositionSave_{};

    Filtered open_{};
    Filtered close_{};

    std::array<Time, 2> hitAt_{};
    std::array<bool, 2> held_{};
    std::array<bool, 2> repeated_{};   ///< this hold has stepped the target

    bool bothHandled_{false};   ///< this hold of both buttons has switched by-hand already
    Time manualUntil_{};

    Target target_{};
    Target savedTarget_{target_};   ///< what flash holds
    Target edited_{};               ///< the target on the display
    bool   editing_{false};
    Time   editUntil_{};
    Time   nextRepeat_{};
};

// -- tests ----------------------------------------------------------------------------------------

namespace Test {
    using namespace Units::literals;
    using namespace std::chrono_literals;

    /// A valve that takes `openTravel` / `closeTravel` end to end, with a Controller on it.
    /// Time moves in 10 ms turns of the loop.
    struct Bench {
        Controller c;
        Time       now{};
        Time       openTravel{20s};
        Time       closeTravel{25s};
        Time       at{};   ///< where the valve really is, as open-run time from closed
        bool       limitsWork{true};
        bool       switchedUnderLoad{false};
        Outputs    last{};
        Time       dt{10};   ///< a turn of the loop
        int        positionSaves{};

        constexpr explicit Bench(Calibration const&      stored         = {},
                                 std::optional<Position> storedPosition = {},
                                 Target                  storedTarget   = Off)
          : c{stored,
              storedPosition,
              storedTarget} {}

        constexpr Actions turn() {
            now += dt;
            auto const o = c.outputs();
            if(o.power && last.power && o.direction != last.direction) { switchedUnderLoad = true; }
            if(o.power != last.power && o.direction != last.direction) { switchedUnderLoad = true; }
            last = o;
            if(o.power) {
                // closing covers openTravel's worth of valve in closeTravel
                auto const step = o.direction ? dt : dt * openTravel.count() / closeTravel.count();
                at              = o.direction ? at + step : at - step;
                if(at > openTravel) { at = openTravel; }
                if(at < Time{}) { at = Time{}; }
            }
            auto const a = c.tick(
              {.open = limitsWork && at >= openTravel, .close = limitsWork && at <= Time{}},
              now);
            if(a.savePosition) { ++positionSaves; }
            return a;
        }

        /// Runs the loop for `d`; true if a save was asked for.
        constexpr bool run(Time d) {
            bool       save = false;
            auto const end  = now + d;
            while(now < end) { save = turn().save || save; }
            return save;
        }

        /// A short press.
        constexpr void tap(Button b) {
            c.button(b, ButtonEvent::hit, now);
            run(100ms);
            c.button(b, ButtonEvent::releaseShort, now);
        }

        /// From idle, the remote control's long press that runs the motor by hand; otherwise the short press
        /// that stops it or acknowledges a fault.
        constexpr void press(Button b) {
            if(c.mode() != Mode::idle) {
                tap(b);
                return;
            }
            c.button(b, ButtonEvent::hit, now);
            c.button(b, ButtonEvent::longPress, now);
            c.button(b, ButtonEvent::releaseLong, now);
        }

        constexpr void holdBoth() {
            c.button(Button::green, ButtonEvent::hit, now);
            c.button(Button::red, ButtonEvent::hit, now);
            run(3s);
            c.button(Button::green, ButtonEvent::longPress, now);
            c.button(Button::red, ButtonEvent::longPress, now);
            run(100ms);
            c.button(Button::green, ButtonEvent::releaseLong, now);
            c.button(Button::red, ButtonEvent::releaseLong, now);
        }
    };

    inline constexpr Calibration Stored{.open = TravelTime{20'000}, .close = TravelTime{25'000}};

    static_assert(!Controller{}.calibrated() && Controller{Stored}.calibrated());
    static_assert(
      !Controller{
        Calibration{.open  = TravelTime{20'000},
                    .close = TravelTime{5}}
    }
         .calibrated(),
      "a travel no actuator has is not used");

    static_assert(
      [] {
          Bench b{};
          b.at = 7s;
          b.dt = 50ms;
          if(b.c.mode() != Mode::calibrating) { return false; }
          bool const saved = b.run(80s);
          auto const cal   = b.c.calibration();
          return saved && b.c.mode() == Mode::idle && cal.open >= 20'000ms && cal.open <= 20'200ms
              && cal.close >= 25'000ms && cal.close <= 25'200ms && Units::same(b.c.percent(), 0_pct)
              && !b.switchedUnderLoad;
      }(),
      "without a calibration at boot the valve measures one by itself");

    static_assert(
      [] {
          Bench b{};
          b.at         = 7s;
          b.limitsWork = false;
          b.press(Button::red);   // stops the calibration
          b.run(2s);
          return b.c.mode() == Mode::idle && !b.c.calibrated() && b.c.outputs() == Outputs{};
      }(),
      "a button stops the calibration from boot, and it does not start again");

    static_assert(
      [] {
          Bench b{
            Calibration{.open = TravelTime{30'000}, .close = TravelTime{30'000}}
          };
          b.at = 7s;
          b.dt = 50ms;
          b.holdBoth();
          if(b.c.mode() != Mode::calibrating) { return false; }
          bool const saved = b.run(80s);
          auto const cal   = b.c.calibration();
          // measured with the limit filter's delay on top
          return saved && b.c.mode() == Mode::idle && cal.open >= 20'000ms && cal.open <= 20'200ms
              && cal.close >= 25'000ms && cal.close <= 25'200ms && Units::same(b.c.percent(), 0_pct)
              && !b.switchedUnderLoad;
      }(),
      "both buttons held: calibration homes, times both runs, saves, and ends closed at 0 %");

    static_assert(
      [] {
          Bench b{Stored};
          if(b.c.percent()) { return false; }   // not referenced yet
          b.press(Button::red);
          b.run(1s);
          if(!Units::same(b.c.percent(), 0_pct)) { return false; }   // stood at the close limit
          b.press(Button::green);
          b.run(10s);
          auto const moving = b.c.view(b.now);
          b.press(Button::green);   // stop
          b.run(1s);
          auto const p = b.c.percent();
          return moving.kind == View::Kind::position && moving.blink && p && *p >= 49_pct
              && *p <= 52_pct && b.c.mode() == Mode::idle && b.c.outputs() == Outputs{}
              && b.c.view(b.now).kind == View::Kind::position
              && b.c.view(b.now + 5s).kind == View::Kind::temperature;
      }(),
      "half the open travel is 50 %, shown while moving and a moment after");

    static_assert(
      [] {
          Bench b{Stored};
          b.press(Button::red);
          b.run(1s);
          b.press(Button::green);
          b.run(30s);
          return Units::same(b.c.percent(), 100_pct) && b.c.mode() == Mode::idle
              && b.c.outputs() == Outputs{};
      }(),
      "a run ends at its limit, at 100 %");

    static_assert(
      [] {
          Bench b{Stored};
          b.at         = 5s;
          b.limitsWork = false;
          b.press(Button::green);
          b.run(60s);
          bool const stopped = b.c.mode() == Mode::fault && b.c.fault() == Fault::timeout
                            && b.c.outputs() == Outputs{}
                            && b.c.view(b.now).kind == View::Kind::fault;
          b.press(Button::red);   // acknowledges, does not move
          b.run(1s);
          return stopped && b.c.mode() == Mode::idle && b.c.outputs() == Outputs{};
      }(),
      "a limit that never comes stops the motor: fault, until a button");

    static_assert(
      [] {
          Bench b{};
          b.at         = 5s;
          b.dt         = 250ms;
          b.limitsWork = false;
          b.press(Button::red);   // stops the calibration from boot
          b.run(1s);
          b.press(Button::green);
          b.run(Controller::MaxTravel + 5s);
          return b.c.mode() == Mode::fault && b.c.outputs() == Outputs{};
      }(),
      "without a calibration MaxTravel is the limit");

    static_assert(
      [] {
          Bench b{Stored};
          b.at = 10s;
          b.press(Button::green);
          b.run(2s);
          b.press(Button::green);   // stop
          b.press(Button::red);     // and straight back
          b.run(3s);
          return b.c.outputs().power && !b.c.outputs().direction && !b.switchedUnderLoad;
      }(),
      "reversing waits for the relays");

    static_assert(
      [] {
          Bench b{Stored};
          b.at = 10s;
          b.c.button(Button::green, ButtonEvent::hit, b.now);
          b.c.button(Button::green, ButtonEvent::releaseShort, b.now + 2ms);
          b.run(1s);
          return b.c.mode() == Mode::idle && !b.c.outputs().power;
      }(),
      "a 2 ms press is a bounce");

    static_assert(
      [] {
          Controller c{Stored};
          Time       now{};
          // a 5 ms spike on the open input while idle, then both inputs for good
          c.tick({.open = true}, now += 5ms);
          c.tick({}, now += 5ms);
          bool const ignored = c.mode() == Mode::idle;
          for(int i = 0; i < 10; ++i) { c.tick({.open = true, .close = true}, now += 10ms); }
          return ignored && c.mode() == Mode::fault && c.fault() == Fault::bothLimits;
      }(),
      "the limit inputs are filtered; both at once is a fault");

    static_assert(
      [] {
          Bench b{Stored};
          b.openTravel = 22s;   // the valve got slower since it was calibrated
          b.dt         = 100ms;
          b.press(Button::red);
          b.run(1s);
          b.press(Button::green);
          bool const saved = b.run(30s);
          auto const first = b.c.calibration().open;
          // half way from 20 s to the 22 s (and the limit filter) this run took
          bool const moved = saved && first >= 21'000ms && first <= 21'150ms
                          && b.c.calibration().close == Stored.close
                          && Units::same(b.c.percent(), 100_pct);
          for(int i = 0; i < 3; ++i) {
              b.press(Button::red);
              b.run(30s);
              b.press(Button::green);
              b.run(30s);
          }
          auto const settled = b.c.calibration().open;
          return moved && settled >= 21'850ms && settled <= 22'250ms && b.c.mode() == Mode::idle;
      }(),
      "a limit that comes later than the position said corrects the travel time");

    static_assert(
      [] {
          Bench b{Stored};
          b.press(Button::red);
          b.run(1s);
          b.press(Button::green);
          b.run(10s);
          b.press(Button::green);   // stop at 50 %
          b.run(1s);
          b.press(Button::green);
          bool const saved = b.run(15s);               // the rest, as long as expected
          auto const open  = b.c.calibration().open;   // the limit filter's 40 ms show
          return !saved && open >= Stored.open && open <= Stored.open + 50ms
              && Units::same(b.c.percent(), 100_pct);
      }(),
      "a limit that comes when expected leaves the flash alone");

    static_assert(
      [] {
          Bench b{Stored};
          b.press(Button::red);
          b.run(1s);
          int const atLimit = b.positionSaves;   // the close limit: 0 %
          b.press(Button::green);
          b.run(5s + 550ms);
          int const running = b.positionSaves - atLimit;
          b.press(Button::green);   // stop
          b.run(5s);
          int const stopped = b.positionSaves - atLimit - running;
          return atLimit == 1 && running >= 4 && running <= 6 && stopped == 1;
      }(),
      "the position goes to flash once a second while it moves, and when it stopped");

    static_assert(
      [] {
          Bench b{Stored, Controller::Full * 3 / 10};
          b.at = 6s;
          if(!Units::same(b.c.percent(), 30_pct) || b.positionSaves != 0) { return false; }
          b.press(Button::green);
          b.run(4s);
          auto const p = b.c.percent();
          return p && *p >= 49_pct && *p <= 52_pct;
      }(),
      "the position from flash is where the valve is after boot");

    static_assert(!Controller{Calibration{},
                              Controller::Full / 2}
                     .percent(),
                  "a stored position without a calibration is nothing");

    static_assert(
      [] {
          Bench b{Stored};
          b.at = 10s;
          b.c.button(Button::green, ButtonEvent::hit, b.now);
          b.run(3s);
          b.c.button(Button::green, ButtonEvent::longPress, b.now);
          b.run(1s);
          return b.c.mode() == Mode::idle && !b.c.outputs().power
              && b.c.view(b.now).kind == View::Kind::target;
      }(),
      "one button held long neither calibrates nor runs the motor: it is the target");

    static_assert(
      [] {
          Bench b{Stored};
          b.at = 10s;
          b.press(Button::green);
          b.run(1s);
          return b.c.mode() == Mode::moving && b.c.outputs().power && b.c.outputs().direction;
      }(),
      "the remote control's long press runs the motor by hand");

    static_assert(
      [] {
          Bench b{Stored, Controller::Full * 3 / 10};
          b.at            = 6s;
          auto const both = [&](ButtonEvent e) {
              b.c.button(Button::green, e, b.now);
              b.c.button(Button::red, e, b.now);
          };
          both(ButtonEvent::hit);
          b.run(1'200ms);
          auto const entering = b.c.view(b.now);
          both(ButtonEvent::releaseShort);
          b.run(500ms);
          bool const in = b.c.mode() == Mode::manual && !b.c.outputs().power
                       && entering.kind == View::Kind::manual && entering.blink
                       && b.c.view(b.now) == View{.kind = View::Kind::manual};

          b.c.button(Button::green, ButtonEvent::hit, b.now);
          b.run(200ms);
          bool const waits = !b.c.outputs().power;   // not before ManualStartDelay
          b.run(4s);
          bool const opens = b.c.outputs().power && b.c.outputs().direction
                          && b.c.view(b.now).kind == View::Kind::position;
          b.c.button(Button::green, ButtonEvent::releaseLong, b.now);
          b.run(1s);
          auto const up      = b.c.percent();
          bool const stopped = !b.c.outputs().power && b.c.mode() == Mode::manual;

          b.c.button(Button::red, ButtonEvent::hit, b.now);
          b.run(3s);
          bool const closes = b.c.outputs().power && !b.c.outputs().direction;
          b.c.button(Button::red, ButtonEvent::releaseShort, b.now);
          b.run(1s);

          both(ButtonEvent::hit);
          b.run(1'200ms);
          both(ButtonEvent::releaseShort);
          b.run(500ms);
          return in && waits && opens && stopped && up && *up >= 48_pct && *up <= 52_pct && closes
              && b.c.mode() == Mode::idle && !b.c.outputs().power && !b.switchedUnderLoad
              && b.c.view(b.now + 5s).kind == View::Kind::temperature;
      }(),
      "both buttons for a second: by hand - a held button runs the motor, letting go stops it");

    static_assert(
      [] {
          Bench b{Stored, Controller::Full / 2};
          b.at = 10s;
          b.dt = 100ms;
          b.c.button(Button::green, ButtonEvent::hit, b.now);
          b.c.button(Button::red, ButtonEvent::hit, b.now);
          b.run(1'200ms);
          b.c.button(Button::green, ButtonEvent::releaseShort, b.now);
          b.c.button(Button::red, ButtonEvent::releaseShort, b.now);
          bool const in = b.c.mode() == Mode::manual;
          b.run(Controller::ManualTimeout + 1s);
          return in && b.c.mode() == Mode::idle;
      }(),
      "by hand ends by itself");

    static_assert(
      [] {
          Bench b{Stored, Controller::Full * 3 / 10};
          b.at = 6s;
          if(!b.c.goTo(Controller::Full / 2)) { return false; }
          b.run(10s);
          auto const up      = b.c.percent();
          bool const stopped = b.c.mode() == Mode::idle && !b.c.outputs().power;
          if(!b.c.goTo(Controller::Full * 4 / 10)) { return false; }
          b.run(10s);
          auto const down = b.c.percent();
          // the real valve is where the position says: 40 % of 20 s
          return stopped && up && *up >= 50_pct && *up <= 51_pct && down && *down >= 39_pct
              && *down <= 40_pct && b.at >= 7'800ms && b.at <= 8'200ms && b.c.mode() == Mode::idle
              && !b.switchedUnderLoad;
      }(),
      "goTo() ends the run where the travel times say the position is reached");

    static_assert(
      [] {
          Bench b{Stored, Controller::Full * 9 / 10};
          b.at = 17s;   // the valve is further from the limit than the position says
          if(!b.c.goTo(Controller::Full)) { return false; }
          b.run(10s);
          bool const atLimit = Units::same(b.c.percent(), 100_pct) && b.at == b.openTravel;
          Controller unknown{Stored};
          return atLimit && b.c.mode() == Mode::idle && !unknown.goTo(Controller::Full / 2);
      }(),
      "goTo(Full) runs to the limit switch; without a position there is no goTo()");

    static_assert(
      [] {
          Bench b{Stored, Controller::Full * 3 / 10};
          b.at = 6s;
          b.c.goTo(Controller::Full * 8 / 10);
          b.run(3s);
          b.c.stop(b.now);
          b.run(1s);
          auto const p = b.c.percent();
          return b.c.mode() == Mode::idle && !b.c.outputs().power && p && *p > 35_pct
              && *p < 50_pct;
      }(),
      "stop() ends a goTo() where it is");

    static_assert(
      [] {
          Bench b{Stored, std::nullopt, 38.0_degC};
          b.c.button(Button::red, ButtonEvent::hit, b.now);
          b.run(Controller::RepeatDelay + 50ms);
          auto const opened = b.c.view(b.now);   // the target comes up as it is
          b.run(Controller::RepeatDelay + Controller::RepeatPeriod * 3);   // 4 steps up
          b.c.button(Button::red, ButtonEvent::releaseShort, b.now);
          auto const held = b.c.view(b.now).target;
          b.run(4s);
          return opened.kind == View::Kind::target && opened.target == 38.0_degC
              && held == 40.0_degC && b.c.target() == 40.0_degC;
      }(),
      "a button held from the temperature brings the target up and then steps it");

    static_assert(
      [] {
          Bench b{Stored};
          b.at = 10s;
          b.tap(Button::red);   // shows the target: off
          auto const shown = b.c.view(b.now);
          b.tap(Button::red);     // 20.0
          b.tap(Button::red);     // 20.5
          b.tap(Button::red);     // 21.0
          b.tap(Button::green);   // 20.5
          bool const editing = b.c.view(b.now).kind == View::Kind::target
                            && b.c.view(b.now).target == 20.5_degC && b.c.view(b.now).blink
                            && b.c.target() == Off;
          int        saves   = 0;
          for(int i = 0; i < 400; ++i) { saves += b.turn().saveTarget ? 1 : 0; }
          return shown.kind == View::Kind::target && shown.target == Off && editing
              && b.c.target() == 20.5_degC && saves == 1
              && b.c.view(b.now).kind == View::Kind::temperature && !b.c.outputs().power
              && b.c.mode() == Mode::idle;
      }(),
      "short presses set the target; it is taken and saved once, 3 s after the last one");

    static_assert(
      [] {
          Bench b{Stored, std::nullopt, 38.0_degC};
          if(b.c.target() != 38.0_degC) { return false; }
          b.tap(Button::green);   // shows 38.0
          b.c.button(Button::green, ButtonEvent::hit, b.now);
          b.run(Controller::RepeatDelay + Controller::RepeatPeriod * 3 + 50ms);   // 4 steps
          b.c.button(Button::green, ButtonEvent::releaseShort, b.now);
          auto const held = b.c.view(b.now).target;
          b.c.button(Button::green, ButtonEvent::hit, b.now);
          b.run(3s);
          b.c.button(Button::green, ButtonEvent::longPress, b.now);   // no motor from in here
          b.run(3s);
          b.c.button(Button::green, ButtonEvent::releaseLong, b.now);
          b.run(4s);
          return held == 36.0_degC && b.c.target() == Off && b.c.mode() == Mode::idle
              && !b.c.outputs().power;
      }(),
      "a held button keeps stepping, down to off, and its release is not one step more");

    static_assert(
      [] {
          Controller c{Stored, std::nullopt, Units::target(9'999)};
          c.setTarget(45.0_degC);
          bool const taken = c.target() == 45.0_degC && c.tick({}, 10ms).saveTarget;
          c.setTarget(1.5_degC);
          return taken && c.target() == 45.0_degC && !c.tick({}, 20ms).saveTarget;
      }(),
      "a target that is none is not taken, from flash or from the remote");
}   // namespace Test

}   // namespace Valve
