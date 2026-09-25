#pragma once

#include "Valve.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <utility>

/// Holds the mixed water at the target temperature, free of the SDK like Valve.hpp: the
/// reading, the target and what the valve is doing in; a Command for Valve::Controller, what
/// happened (Events, for the log) and a Status out. Tested against a simulated plant at the end.
///
/// The plant: the valve sits in the COLD line in front of a mix point with boiler water (up to
/// 90 degC in summer, cold ~16 degC), the sensor in the water 20 cm behind it; a fixed bypass
/// always lets some cold through. The water follows the valve within seconds - what is slow is
/// the motor (70 s end to end), so this is a three-point step regulator: a run in proportion
/// to the error, a wait for the sensor, another look. The position that fits depends on the
/// boiler, which changes over hours: between draws the valve stays where it was.
///
/// Without flow the reading says nothing about the valve, and there is no flow sensor. Water
/// is taken to flow for Parameter::flowHold after the reading moved fast (a standing pipe only
/// drifts) or answered a run. Then both directions regulate. Otherwise:
///  - too hot acts all the same, one run every hotRetry: never returned, colder is the safe side;
///  - too cold acts only on a reading that is steady and above probeMinTemp (a standing pipe
///    falls, and ends at room temperature), with one run every probePeriod that is taken back
///    when nothing answers.
/// responseTries runs in a row without an answer end "water flows"; those towards hot are
/// taken back. From fastError too hot the run is a long one (fastStep), ended when the reading
/// comes down to fastExit, and followed by the next without a wait while it answers.
///
/// Every number is a Parameter: changed over the remote control, kept in flash by main().
///
/// What is a property of the pipework rather than a choice is learned (Learned, Parameter::learn):
///  - the valve's curve: travel per kelvin, by region of the travel, from every clean run - one
///    made with water flowing and the reading at rest before and after. A run is then as long
///    as the curve says (times learnUse: short of it rather than over);
///  - the backlash, from what the first run after a change of direction falls short;
///  - which way the valve acts: a reading that answers runs the wrong way flips coldAtOpen.
/// The wait after a run is not learned but adaptive: from settle until the reading has come
/// to rest, settleMax at the most.
namespace Regulator {

using Units::Gain;
using Units::Kelvins;
using Units::Position;
using Units::Rate;
using Units::Target;
using Units::Temperature;
using Valve::Time;

/// What flash holds of what was learned: 16 bits each, the flash format.
using StoredGain   = Units::mpu::quantity<Gain::unit, std::uint16_t>;
using StoredTravel = Units::mpu::quantity<Units::deciPercent, std::uint16_t>;

// -- parameters -----------------------------------------------------------------------------------

enum class Parameter : std::uint8_t {
    enabled,
    coldAtOpen,
    bandHot,
    bandCold,
    bandInner,
    gain,
    minStep,
    maxStep,
    backlash,
    settle,
    fastError,
    fastExit,
    fastStep,
    flowRate,
    fastWindow,
    flowHold,
    responseMin,
    responseTries,
    hotRetry,
    probePeriod,
    probeMinTemp,
    steadyRate,
    slowWindow,
    filter,
    settleMax,
    learn,
    learnUse,
    slowFlow,
    quietMax,
    count_
};

inline constexpr std::size_t ParameterCount = std::to_underlying(Parameter::count_);

struct ParameterInfo {
    std::string_view name;
    std::string_view unit;
    std::int16_t     initial;
    std::int16_t     min;
    std::int16_t     max;
    std::string_view help;
};

// The numbers in the table are plain int16 - one list for flash, the wire and the tui - and
// each parameter's unit is an mp-units unit: the column is its symbol, and the Controller
// reads a parameter as the quantity it is (kelvins_(), travel_() ...).
namespace U {
    using Units::symbol;
    namespace si               = Units::si;
    inline constexpr auto none = std::string_view{};
    inline constexpr auto dK   = symbol<Units::deciK>();
    inline constexpr auto dPct = symbol<Units::deciPercent>();
    inline constexpr auto gain = symbol<Gain::unit>();
    inline constexpr auto ds   = symbol<si::deci<si::second>>();
    inline constexpr auto sec  = symbol<si::second>();
    inline constexpr auto cKs  = symbol<si::centi<si::kelvin> / si::second>();
    inline constexpr auto mKs  = symbol<Rate::unit>();
    inline constexpr auto dC   = symbol<Units::deciDegC>();
    inline constexpr auto pct  = symbol<Units::mpu::percent>();
}   // namespace U

inline constexpr std::array<ParameterInfo, ParameterCount> Info{
  {
   {"enabled", U::none, 1, 0, 1, "0: the regulator never moves the valve"},
   {"coldAtOpen", U::none, 1, 0, 1, "1: 100 % (open limit) is the most cold water, 0: the least"},
   {"bandHot", U::dK, 30, 5, 300, "this far above the target a correction starts"},
   {"bandCold", U::dK, 40, 5, 300, "this far below the target a correction starts"},
   {"bandInner", U::dK, 15, 0, 200, "a correction goes on until the reading is this close"},
   {"gain", U::gain, 200, 5, 1000, "travel per kelvin of error, in 0.01 %"},
   {"minStep", U::dPct, 7, 1, 200, "the shortest run"},
   {"maxStep", U::dPct, 150, 5, 1000, "the longest run (not the fast one)"},
   {"backlash", U::dPct, 0, 0, 200, "added to the first run after a change of direction"},
   {"settle", U::ds, 150, 5, 6000, "wait after a run before the reading is judged"},
   {"fastError", U::dK, 100, 20, 900, "this far too hot: long runs, no waits"},
   {"fastExit", U::dK, 50, 0, 900, "a fast run ends when the reading is down to this"},
   {"fastStep", U::dPct, 250, 10, 1000, "a fast run's length"},
   {"flowRate", U::cKs, 15, 1, 1000, "the reading moving this fast means water flows"},
   {"fastWindow", U::ds, 40, 8, 600, "... measured over this"},
   {"flowHold", U::sec, 60, 5, 3600, "water is taken to flow this long after a sign of it"},
   {"responseMin", U::dK, 5, 1, 200, "a run this much answered by the reading is a sign of flow"},
   {"responseTries", U::none, 2, 1, 50, "runs without an answer until 'no flow'"},
   {"hotRetry", U::sec, 120, 5, 3600, "too hot without flow: one run this often"},
   {"probePeriod",
     U::sec,
     0,
     0,
     30000,
     "too cold, steady, without flow: one run this often; 0 never"},
   {"probeMinTemp", U::dC, 400, 0, 1000, "... only above this (a standing pipe ends colder)"},
   {"steadyRate", U::mKs, 4, 1, 1000, "... and the reading slower than this"},
   {"slowWindow", U::sec, 60, 8, 1800, "... measured over this"},
   {"filter", U::ds, 5, 0, 100, "the reading's low pass"},
   {"settleMax",
     U::ds,
     450,
     5,
     6000,
     "... but no longer than this for the reading to come to rest"},
   {"learn",
     U::none,
     2,
     0,
     2,
     "0: nothing is learned, 1: learned and logged only, 2: learned and used"},
   {"learnUse", U::pct, 70, 10, 150, "how much of the learned travel per kelvin a run takes"},
   {"slowFlow",
     U::mKs,
     3,
     0,
     200,
     "a rise held this fast for half a minute means water flows too; 0 never"},
   {"quietMax",
     U::ds,
     600,
     0,
     3000,
     "a run waits for the reading to be at rest, but no longer than this; 0 never"},
   }
};

using Values = std::array<std::int16_t, ParameterCount>;

[[nodiscard]] constexpr Values defaults() {
    Values v{};
    for(std::size_t i = 0; i < ParameterCount; ++i) { v[i] = Info[i].initial; }
    return v;
}

/// The flash format. A firmware with other parameters has another Version and starts from
/// defaults().
inline constexpr std::uint16_t Version = 3;

struct Stored {
    std::uint16_t version{};
    Values        values{};

    constexpr bool operator==(Stored const&) const = default;
};

/// What the regulator has found out about the valve; kept in flash by main(). The layout is
/// the flash format.
inline constexpr std::size_t   Bins           = 8;   ///< regions of the travel
inline constexpr std::uint16_t LearnedVersion = 1;

struct Learned {
    std::uint16_t                  version{};
    std::array<StoredGain, Bins>   gain{};      ///< travel per kelvin; 0: nothing yet
    std::array<std::uint8_t, Bins> samples{};   ///< clean runs that went into it
    StoredTravel                   backlash{};
    std::uint8_t                   backlashSamples{};

    constexpr bool operator==(Learned const&) const = default;
};

// -- out ------------------------------------------------------------------------------------------

enum class State : std::uint8_t {
    off,       ///< not enabled, or no target
    blocked,   ///< the valve is somebody else's: calibration, fault, by hand, the target on the display
    blind,      ///< no reading
    homing,     ///< position not known: to the cold limit
    watching,   ///< nothing to do
    moving,     ///< a run of mine
    settling,   ///< waiting for the reading to follow
};

struct Command {
    enum class Kind : std::uint8_t { none, goTo, stop, run };

    Kind             kind{Kind::none};
    Position         position{};    ///< goTo
    Valve::Direction direction{};   ///< run
};

struct Event {
    enum class Kind : std::uint8_t {
        state,        ///< n: new State, m: old
        flowStart,    ///< rate: the slope that showed it, n: 1 when it was the slow one
        flowEnd,      ///< since: the last sign of flow
        run,          ///< from, to, kelvins: the error
        fastRun,      ///< same
        probe,        ///< same: a run to find out whether water flows
        fastStop,     ///< from: the position, kelvins: the error
        response,     ///< kelvins: the reading's change, from: the position
        noResponse,   ///< kelvins: the change, n: runs without an answer, m: 1 the flow lapsed
        takeBack,     ///< from, to
        kept,         ///< from: the position kept because the flow lapsed while we waited
        noFlow,       ///< n: runs without an answer
        limit,   ///< from: the position, kelvins: the error - at the end of the travel and still off
        parameters,        ///< changed
        learnedGain,       ///< n: region of the travel, measured, gain: the region's value now
        learnRejected,     ///< n: region, measured, gain: what it was held against
        learnedBacklash,   ///< from: measured, to: the value now
        wrongWay,          ///< kelvins: the reading's change, n: times in a row
        direction,         ///< n: coldAtOpen as it is now - the valve acts the other way round
    };

    Kind                    kind{};
    std::optional<Position> from{};   ///< empty: not known
    Position                to{};
    Kelvins                 kelvins{};
    Rate                    rate{};
    Time                    since{};
    Gain                    measured{};
    Gain                    gain{};
    std::int32_t            n{};
    std::int32_t            m{};
};

struct Events {
    std::array<Event, 8> list{};
    std::size_t          count{};

    constexpr void add(Event e) {
        if(count < list.size()) { list[count++] = e; }
    }
};

struct Status {
    State         state{};
    bool          flowing{};
    bool          correcting{};
    bool          fast{};
    bool          atLimit{};       ///< at the end of the travel and still off the target
    std::uint8_t  noResponses{};   ///< runs in a row without an answer
    Temperature   reading{};       ///< filtered
    Kelvins       error{};         ///< reading - target; 0 without either
    Rate          fastSlope{};
    Rate          slowSlope{};
    std::uint32_t runs{};   ///< runs asked of the valve since boot: the relays' wear

    constexpr bool operator==(Status const&) const = default;
};

struct Input {
    std::optional<Temperature> reading;
    Target                     target{};   ///< Valve::Off: none
    std::optional<Position>    position;   ///< Valve::Controller::position()
    Valve::Mode                mode{};
    bool                       editing{};
    bool                       calibrated{};
};

struct Output {
    Command command{};
    Events  events{};
    bool    saveParameters{};   ///< stored() to flash
    bool    saveLearned{};      ///< learned() to flash
};

// -- the regulator --------------------------------------------------------------------------------

/// (newest - oldest) over a window of 8 sample periods.
class Slope {
public:
    constexpr void reset() { count_ = 0; }

    constexpr void feed(Temperature reading,
                        Time        now,
                        Time        window) {
        auto const period = window / Intervals;
        if(count_ != 0 && now - last_ < period) { return; }
        // a gap (no reading for a while): what is in there is not one window any more
        if(count_ != 0 && now - last_ > period * 3) { count_ = 0; }
        last_ = now;
        for(std::size_t i = samples_.size() - 1; i > 0; --i) { samples_[i] = samples_[i - 1]; }
        samples_[0] = {reading, now};
        if(count_ < samples_.size()) { ++count_; }
    }

    /// Empty until the window is full.
    [[nodiscard]] constexpr std::optional<Rate> value() const {
        if(count_ < samples_.size()) { return std::nullopt; }
        auto const& newest = samples_.front();
        auto const& oldest = samples_.back();
        auto const  ms     = (newest.at - oldest.at).count();
        if(ms <= 0) { return std::nullopt; }
        // per second from per millisecond, in 64 bits
        namespace mpu      = Units::mpu;
        Kelvins const rise = newest.reading - oldest.reading;
        return mpu::value_cast<std::int32_t>(mpu::value_cast<std::int64_t>(rise) * 1000 / ms)
             / (1 * Units::si::second);
    }

private:
    static constexpr int Intervals = 8;

    struct Sample {
        Temperature reading{};
        Time        at{};
    };

    std::array<Sample, Intervals + 1> samples_{};
    std::size_t                       count_{};
    Time                              last_{};
};

class Controller {
public:
    static constexpr Position Full   = Units::Full;
    static constexpr Position Closed = Units::Closed;
    /// Changed parameters go to flash this long after the last change.
    static constexpr Time SaveDelay{5'000};

    /// What was learned goes to flash this often at the most, and only when it changed.
    static constexpr Time LearnedSavePeriod{3'600'000};
    static constexpr Time FirstLearnedSave{600'000};
    /// A region's learned value is used from this many clean runs on.
    static constexpr std::uint8_t MinSamples = 2;
    /// A measured travel per kelvin further than this factor from the gain parameter is not
    /// learned from.
    static constexpr std::int32_t LearnTrust = 3;
    /// A region's first sample has no learned value to be held against: the window around the
    /// gain parameter is this wide instead, or a gain set wrong once could never be corrected
    /// (2026-09-20: the only clean run of a night measured 357 c%/K against a parameter of 80).
    static constexpr std::int32_t FirstLearnTrust = 10;
    /// A slow rise counts as a sign of flow once it has held this long. Measured 2026-09-20:
    /// a standing pipe managed 7 s of it in 16 119 s, the circulation pump 25 minutes.
    static constexpr Time SlowFlowFor{30'000};
    /// A reading that answers this many runs in a row the wrong way round: coldAtOpen is wrong.
    static constexpr std::uint8_t WrongWayFlips = 2;

    constexpr explicit Controller(Stored const&  stored  = {},
                                  Learned const& learned = {})
      : values_{stored.version == Version ? clamped_(stored.values) : defaults()}
      , learned_{learned.version == LearnedVersion ? learned : Learned{.version = LearnedVersion}} {
    }

    [[nodiscard]] constexpr Learned const& learned() const { return learned_; }

    constexpr void resetLearned(Time now) {
        learned_         = Learned{.version = LearnedVersion};
        learnedDirty_    = true;
        nextLearnedSave_ = now + SaveDelay;
    }

    // -- parameters -------------------------------------------------------------------------------

    [[nodiscard]] constexpr Values const& values() const { return values_; }

    [[nodiscard]] constexpr Stored stored() const {
        return {.version = Version, .values = values_};
    }

    [[nodiscard]] constexpr std::int32_t get(Parameter p) const {
        return values_[std::to_underlying(p)];
    }

    /// Clamped to the parameter's range. True when it changed.
    constexpr bool set(std::size_t  index,
                       std::int16_t value,
                       Time         now) {
        if(index >= ParameterCount) { return false; }
        auto const& info = Info[index];
        value            = value < info.min ? info.min : (value > info.max ? info.max : value);
        if(values_[index] == value) { return false; }
        values_[index] = value;
        changed_(now);
        return true;
    }

    constexpr void resetParameters(Time now) {
        values_ = defaults();
        changed_(now);
    }

    // -- every turn of the main loop --------------------------------------------------------------

    constexpr Output tick(Input const& in,
                          Time         now) {
        Output out{};
        if(parametersChanged_) {
            parametersChanged_ = false;
            out.events.add({.kind = Event::Kind::parameters});
        }
        if(saveAt_ && now >= *saveAt_) {
            saveAt_.reset();
            out.saveParameters = true;
        }
        if(learnedDirty_ && now >= nextLearnedSave_) {
            learnedDirty_    = false;
            nextLearnedSave_ = now + LearnedSavePeriod;
            out.saveLearned  = true;
        }

        measure_(in, now, out.events);
        auto const next = step_(in, now, out);
        if(next != state_) {
            out.events.add({.kind = Event::Kind::state,
                            .n    = std::to_underlying(next),
                            .m    = std::to_underlying(state_)});
            state_ = next;
        }
        if(out.command.kind == Command::Kind::goTo || out.command.kind == Command::Kind::run) {
            ++runs_;
        }
        return out;
    }

    [[nodiscard]] constexpr Status status() const {
        return {.state       = state_,
                .flowing     = flowing_,
                .correcting  = correcting_,
                .fast        = fast_,
                .atLimit     = atLimit_,
                .noResponses = noResponses_,
                .reading     = filtered_.value_or(Temperature{}),
                .error       = error_,
                .fastSlope   = fastSlope_.value().value_or(Rate{}),
                .slowSlope   = slowSlope_.value().value_or(Rate{}),
                .runs        = runs_};
    }

    [[nodiscard]] constexpr State state() const { return state_; }

private:
    [[nodiscard]] static constexpr Values clamped_(Values v) {
        for(std::size_t i = 0; i < ParameterCount; ++i) {
            if(v[i] < Info[i].min || v[i] > Info[i].max) { v[i] = Info[i].initial; }
        }
        return v;
    }

    constexpr void changed_(Time now) {
        parametersChanged_ = true;
        saveAt_            = now + SaveDelay;
        fastSlope_.reset();
        slowSlope_.reset();
    }

    // a parameter as the quantity it is: the unit is the one its Info line names
    [[nodiscard]] constexpr Kelvins kelvins_(Parameter p) const {
        return Units::deciKelvins(get(p));
    }

    [[nodiscard]] constexpr Position travel_(Parameter p) const {
        return Units::deciPercents(get(p));
    }

    [[nodiscard]] constexpr Gain gain_() const { return Units::gain(get(Parameter::gain)); }

    [[nodiscard]] constexpr Rate flowRate_() const {
        return Units::rate(get(Parameter::flowRate) * 10);   // cK/s
    }

    [[nodiscard]] constexpr Rate slowFlowRate_() const {
        return Units::rate(get(Parameter::slowFlow));
    }

    [[nodiscard]] constexpr Rate steadyRate_() const {
        return Units::rate(get(Parameter::steadyRate));
    }

    [[nodiscard]] constexpr Temperature probeMinTemp_() const {
        return Units::target(static_cast<std::int16_t>(get(Parameter::probeMinTemp)));
    }

    [[nodiscard]] constexpr Time seconds_(Parameter p) const { return Time{get(p) * 1000}; }

    [[nodiscard]] constexpr Time tenthSeconds_(Parameter p) const { return Time{get(p) * 100}; }

    /// The filtered reading, the slopes, and whether water flows.
    constexpr void measure_(Input const& in,
                            Time         now,
                            Events&      events) {
        if(!in.reading) {
            filtered_.reset();
            fastSlope_.reset();
            slowSlope_.reset();
        } else if(!filtered_) {
            filtered_   = *in.reading;
            filteredAt_ = now;
        } else {
            // a first-order low pass in steps of 100 ms
            while(now - filteredAt_ >= FilterStep) {
                filteredAt_ += FilterStep;
                *filtered_ += (*in.reading - *filtered_) / (1 + get(Parameter::filter));
            }
        }
        if(filtered_) {
            fastSlope_.feed(*filtered_, now, tenthSeconds_(Parameter::fastWindow));
            slowSlope_.feed(*filtered_, now, seconds_(Parameter::slowWindow));
        }
        error_ = filtered_ && in.target != Valve::Off ? Kelvins{*filtered_ - in.target} : Kelvins{};

        if(auto const slope = fastSlope_.value()) {
            if(Units::magnitude(*slope) >= flowRate_()) { sign_(now, *slope, events); }
        }
        // The circulation pump does not arrive as a step the fast slope could see: it warms
        // the whole loop over half an hour (2026-09-20: +8 K in 25 min, 3..9 mK/s, while the
        // fast slope never left the noise). A pipe that stands can only cool, so a rise held
        // this long can only be water moving.
        if(auto const slow = slowSlope_.value();
           get(Parameter::slowFlow) != 0 && slow && *slow >= slowFlowRate_())
        {
            if(!slowRiseSince_) { slowRiseSince_ = now; }
            if(now - *slowRiseSince_ >= SlowFlowFor) { sign_(now, *slow, events, true); }
        } else {
            slowRiseSince_.reset();
        }
        if(!atRest_()) {
            if(!restlessSince_) { restlessSince_ = now; }
        } else {
            restlessSince_.reset();
        }
        if(flowing_ && now - signAt_ >= seconds_(Parameter::flowHold)) {
            flowing_ = false;
            events.add({.kind = Event::Kind::flowEnd, .since = now - signAt_});
        }
    }

    /// A sign that water flows.
    constexpr void sign_(Time    now,
                         Rate    slope,
                         Events& events,
                         bool    slow = false) {
        signAt_      = now;
        noResponses_ = 0;
        if(!flowing_) {
            flowing_ = true;
            events.add({.kind = Event::Kind::flowStart, .rate = slope, .n = slow ? 1 : 0});
        }
    }

    [[nodiscard]] constexpr bool towardsCold_(Position from,
                                              Position to) const {
        return (to > from) == (get(Parameter::coldAtOpen) != 0);
    }

    constexpr State step_(Input const& in,
                          Time         now,
                          Output&      out) {
        bool const mine = state_ == State::moving || state_ == State::homing;

        if(get(Parameter::enabled) == 0 || in.target == Valve::Off) {
            quiet_();
            return State::off;
        }
        if(!in.calibrated || in.editing || in.mode == Valve::Mode::calibrating
           || in.mode == Valve::Mode::fault || in.mode == Valve::Mode::manual
           || (in.mode == Valve::Mode::moving && !mine))
        {
            quiet_();
            return State::blocked;
        }

        if(mine && in.mode == Valve::Mode::moving) {
            // a fast run ends as soon as the reading is down
            if(state_ == State::moving && fast_ && filtered_
               && error_ <= kelvins_(Parameter::fastExit))
            {
                out.command = {.kind = Command::Kind::stop};
                out.events.add(
                  {.kind = Event::Kind::fastStop, .from = in.position, .kelvins = error_});
            }
            return state_;
        }
        if(state_ == State::homing) { return in.position ? State::watching : State::blocked; }
        if(state_ == State::moving) {
            // the run is over (or a button ended it)
            settleUntil_ = now + tenthSeconds_(Parameter::settle);
            settleMax_   = now + tenthSeconds_(Parameter::settleMax);
            if(fast_ && filtered_ && answered_() && error_ >= kelvins_(Parameter::fastError)) {
                settleUntil_ = now;   // it answers and is still far off: the next one at once
                settleMax_   = now;
            }
            return State::settling;
        }

        if(!filtered_) {
            quiet_();
            return State::blind;
        }
        if(!in.position) {
            out.command = {.kind      = Command::Kind::run,
                           .direction = get(Parameter::coldAtOpen) != 0 ? Valve::Direction::open
                                                                        : Valve::Direction::close};
            return State::homing;
        }

        if(state_ == State::settling) {
            if(now < settleUntil_) { return State::settling; }
            // the reading is still on its way: what the run did cannot be said yet
            bool const rest = atRest_();
            if(!rest && now < settleMax_) { return State::settling; }
            if(judge_(*in.position, now, rest, out)) { return State::moving; }
        }
        return decide_(*in.position, now, out);
    }

    constexpr void quiet_() {
        correcting_ = false;
        fast_       = false;
        judging_    = false;
        atLimit_    = false;
    }

    /// The reading moves slower than half of what counts as a sign of flow.
    [[nodiscard]] constexpr bool atRest_() const {
        auto const slope = fastSlope_.value();
        return slope && Units::magnitude(*slope) < flowRate_() / 2;
    }

    [[nodiscard]] static constexpr std::size_t bin_(Position position) {
        auto const at = Units::raw(position);
        auto const b  = static_cast<std::size_t>(at < 0 ? 0 : at) * Bins
                      / (static_cast<std::size_t>(Units::raw(Full)) + 1);
        return b < Bins ? b : Bins - 1;
    }

    /// The range the gain parameter may have, which what is learned keeps to as well.
    [[nodiscard]] static constexpr Gain gainInRange_(Gain g) {
        auto const& info = Info[std::to_underlying(Parameter::gain)];
        return Units::clamp(g, Units::gain(info.min), Units::gain(info.max));
    }

    /// Travel per kelvin at `position`: the learned one where there is one.
    [[nodiscard]] constexpr Gain gainAt_(Position position) const {
        auto const b = bin_(position);
        if(get(Parameter::learn) < 2 || learned_.samples[b] < MinSamples) { return gain_(); }
        return gainInRange_(Gain{learned_.gain[b]} * get(Parameter::learnUse) / 100);
    }

    [[nodiscard]] constexpr Position backlash_() const {
        if(get(Parameter::learn) < 2 || learned_.backlashSamples < MinSamples) {
            return travel_(Parameter::backlash);
        }
        return learned_.backlash;
    }

    /// A clean run from runFrom_ to `position` changed the reading by `towards` the way it
    /// should: what that says about the valve.
    constexpr void learn_(Position position,
                          Kelvins  towards,
                          Events&  events) {
        namespace mpu    = Units::mpu;
        auto const moved = Units::magnitude(position - runFrom_);
        if(moved == Closed || towards <= Kelvins{} || position <= Closed || position >= Full) {
            return;
        }
        auto const b = bin_((runFrom_ + position) / 2);
        // a quarter of the way to what was measured; the first one is taken as it is
        auto const blend = [](auto old, auto measured, std::uint8_t samples) {
            return samples == 0 ? measured : old + (measured - old) / 4;
        };
        if(!runReversed_) {
            // travel per kelvin: the travel in 1e-5 %, so that the quotient counts in 0.01 %/K
            constexpr auto fine = mpu::mag_power<10, -5> * mpu::percent;
            auto const     g    = gainInRange_(mpu::value_cast<std::int32_t>(
              (mpu::value_cast<std::int64_t>(mpu::value_cast<fine>(moved)) / towards)
                .in(Gain::unit)));
            // Something else moved the reading while the run was judged (the boiler, a second
            // tap): what comes out far from the gain in use says nothing about the valve. The
            // window has to follow what we believe NOW - held against the gain parameter for
            // ever, a parameter set wrong once could never be corrected.
            bool const known     = learned_.samples[b] >= MinSamples;
            Gain const reference = known ? Gain{learned_.gain[b]} : gain_();
            auto const trust     = known ? LearnTrust : FirstLearnTrust;
            if(g * trust < reference || g > reference * trust) {
                events.add({.kind     = Event::Kind::learnRejected,
                            .measured = g,
                            .gain     = reference,
                            .n        = static_cast<std::int32_t>(b)});
                return;
            }
            learned_.gain[b] = mpu::value_cast<std::uint16_t>(
              blend(Gain{learned_.gain[b]}, g, learned_.samples[b]));
            if(learned_.samples[b] < 255) { ++learned_.samples[b]; }
            learnedDirty_ = true;
            events.add({.kind     = Event::Kind::learnedGain,
                        .measured = g,
                        .gain     = learned_.gain[b],
                        .n        = static_cast<std::int32_t>(b)});
        } else if(learned_.samples[b] >= MinSamples) {
            // what the reading says the valve moved, against what the motor ran
            Position const effective
              = mpu::value_cast<Units::milliPercent>(towards * Gain{learned_.gain[b]});
            auto const& info = Info[std::to_underlying(Parameter::backlash)];
            auto const  missing
              = Units::clamp(moved - effective, Closed, Units::deciPercents(info.max));
            learned_.backlash = mpu::value_cast<std::uint16_t>(mpu::value_cast<Units::deciPercent>(
              blend(Position{learned_.backlash}, missing, learned_.backlashSamples)));
            if(learned_.backlashSamples < 255) { ++learned_.backlashSamples; }
            learnedDirty_ = true;
            events.add(
              {.kind = Event::Kind::learnedBacklash, .from = missing, .to = learned_.backlash});
        }
    }

    /// The valve acts the other way round than coldAtOpen says.
    constexpr void flip_(Time    now,
                         Events& events) {
        auto& value = values_[std::to_underlying(Parameter::coldAtOpen)];
        value       = value != 0 ? 0 : 1;
        wrongWay_   = 0;
        lastUp_.reset();
        changed_(now);
        events.add({.kind = Event::Kind::direction, .n = value});
    }

    /// Did the reading follow the last run, by responseMin, the way it should?
    [[nodiscard]] constexpr bool answered_() const {
        Kelvins const change = *filtered_ - runStart_;
        return (runTowardsCold_ ? -change : change) >= kelvins_(Parameter::responseMin);
    }

    /// After the wait: what did the run do? True when a run back was asked for.
    constexpr bool judge_(Position position,
                          Time     now,
                          bool     atRest,
                          Output&  out) {
        if(!judging_) { return false; }
        judging_              = false;
        Kelvins const change  = *filtered_ - runStart_;
        Kelvins const towards = runTowardsCold_ ? -change : change;
        bool const    learn   = get(Parameter::learn) != 0;
        if(answered_()) {
            out.events.add({.kind = Event::Kind::response, .from = position, .kelvins = change});
            sign_(now, Rate{}, out.events);
            wrongWay_ = 0;
            if(learn && runClean_ && atRest) { learn_(position, towards, out.events); }
            return false;
        }
        // An answer, but the wrong way round: water flows, and coldAtOpen may be wrong.
        if(learn && runClean_ && -towards >= 2 * kelvins_(Parameter::responseMin)) {
            if(wrongWay_ < 255) { ++wrongWay_; }
            out.events.add({.kind = Event::Kind::wrongWay, .kelvins = change, .n = wrongWay_});
            sign_(now, Rate{}, out.events);
            if(wrongWay_ >= WrongWayFlips && get(Parameter::learn) == 2) { flip_(now, out.events); }
            return false;
        }
        // The water stopped while we waited: there was nothing left to answer with, which
        // says nothing about the valve.
        bool const lostFlow = flowAtRun_ && !flowing_;
        if(noResponses_ < 255) { ++noResponses_; }
        out.events.add({.kind    = Event::Kind::noResponse,
                        .kelvins = change,
                        .n       = noResponses_,
                        .m       = lostFlow ? 1 : 0});

        bool const given = probing_ || noResponses_ >= get(Parameter::responseTries);
        probing_         = false;
        if(!flowing_) { nextHotRetry_ = now + seconds_(Parameter::hotRetry); }
        if(!given) { return false; }
        if(flowing_) {
            flowing_ = false;
            out.events.add({.kind = Event::Kind::noFlow, .n = noResponses_});
        }
        correcting_   = false;
        nextHotRetry_ = now + seconds_(Parameter::hotRetry);
        // towards hot for nothing: back, or the next draw starts too hot - but not when the
        // water simply stopped while we waited, when what was found is worth keeping.
        if(!runTowardsCold_ && takeBackTo_ != position) {
            if(lostFlow) {
                out.events.add({.kind = Event::Kind::kept, .from = position});
                return false;
            }
            out.events.add({.kind = Event::Kind::takeBack, .from = position, .to = takeBackTo_});
            out.command = {.kind = Command::Kind::goTo, .position = takeBackTo_};
            fast_       = false;
            return true;
        }
        return false;
    }

    constexpr State decide_(Position position,
                            Time     now,
                            Output&  out) {
        auto const inner = kelvins_(Parameter::bandInner);
        auto const limit = [&](Parameter band) {
            return correcting_ && inner < kelvins_(band) ? inner : kelvins_(band);
        };
        bool const hot  = error_ > limit(Parameter::bandHot);
        bool const cold = error_ < -limit(Parameter::bandCold);
        if(!hot && !cold) {
            quiet_();
            return State::watching;
        }

        // The reading is on its way there by itself (a draw that starts on a cold pipe, the
        // last run still working): wait for it. A draw arriving shows in the fast slope, the
        // circulation pump warming the whole loop only in the slow one - without the second
        // test the regulator drives hard towards hot all through the pump's half hour and
        // overshoots when the loop arrives.
        if(auto const slope = fastSlope_.value()) {
            if(hot ? *slope <= -flowRate_() : *slope >= flowRate_()) { return State::watching; }
        }
        // ... and only while the fast slope is quiet: a draw moving the reading is the fast
        // test's and the quiet gate's business, and its own slow slope is far above this.
        if(atRest_()) {
            if(auto const slow = slowSlope_.value(); get(Parameter::slowFlow) != 0 && slow) {
                if(hot ? *slow <= -slowFlowRate_() : *slow >= slowFlowRate_()) {
                    return State::watching;
                }
            }
        }

        // The reading is still moving: what the error says now is not where it will end up.
        // (2026-09-20: a draw's stored slug ran 6 K past the target and back, and the run made
        // on the way up went the wrong way and was then credited with the whole fall.) Wait
        // for it to come to rest, but not for ever.
        if(get(Parameter::quietMax) != 0 && restlessSince_
           && now - *restlessSince_ < tenthSeconds_(Parameter::quietMax))
        {
            return State::watching;
        }

        bool probe = false;
        if(!flowing_) {
            if(hot) {
                if(now < nextHotRetry_) { return State::watching; }
            } else {
                auto const slow = slowSlope_.value();
                probe           = get(Parameter::probePeriod) != 0 && now >= nextProbe_ && slow
                               && Units::magnitude(*slow) <= steadyRate_() && *filtered_ >= probeMinTemp_();
                if(!probe) { return State::watching; }
            }
        }

        fast_ = hot && error_ >= kelvins_(Parameter::fastError);
        // the error, in the tenths of a kelvin the bands count in, times the travel per kelvin
        Position move
          = Units::mpu::value_cast<Units::deciK>(Units::magnitude(error_)) * gainAt_(position);
        if(move > travel_(Parameter::maxStep)) { move = travel_(Parameter::maxStep); }
        if(move < travel_(Parameter::minStep)) { move = travel_(Parameter::minStep); }
        if(fast_) { move = travel_(Parameter::fastStep); }

        bool const up       = hot == (get(Parameter::coldAtOpen) != 0);
        bool const reversed = lastUp_ && *lastUp_ != up;
        if(reversed) { move += backlash_(); }
        auto const to = Units::clamp(up ? position + move : position - move, Closed, Full);
        if(to == position) {
            // the end of the travel: too cold there is a boiler below the target
            if(!atLimit_) {
                atLimit_ = true;
                out.events.add({.kind = Event::Kind::limit, .from = position, .kelvins = error_});
            }
            // driven to the end by a reading that answered the wrong way
            if(wrongWay_ != 0 && get(Parameter::learn) == 2) { flip_(now, out.events); }
            correcting_ = false;
            return State::watching;
        }
        atLimit_ = false;

        if(noResponses_ == 0) { takeBackTo_ = position; }
        if(probe) {
            takeBackTo_ = position;
            nextProbe_  = now + seconds_(Parameter::probePeriod);
        }
        runFrom_        = position;
        runReversed_    = reversed;
        runClean_       = flowing_ && !probe && !fast_ && atRest_();
        flowAtRun_      = flowing_;
        probing_        = probe;
        judging_        = true;
        correcting_     = true;
        lastUp_         = up;
        runStart_       = *filtered_;
        runTowardsCold_ = towardsCold_(position, to);
        out.command     = {.kind = Command::Kind::goTo, .position = to};
        out.events.add({.kind    = probe ? Event::Kind::probe
                                 : fast_ ? Event::Kind::fastRun
                                         : Event::Kind::run,
                        .from    = position,
                        .to      = to,
                        .kelvins = error_});
        return State::moving;
    }

    static constexpr Time FilterStep{100};

    Values              values_{};
    bool                parametersChanged_{false};
    std::optional<Time> saveAt_{};

    State state_{State::off};

    std::optional<Temperature> filtered_{};
    Time                       filteredAt_{};
    Slope                      fastSlope_{};
    Slope                      slowSlope_{};
    Kelvins                    error_{};

    bool                flowing_{false};
    Time                signAt_{};
    std::uint8_t        noResponses_{};
    std::optional<Time> slowRiseSince_{};   ///< the slow slope has been rising since
    std::optional<Time> restlessSince_{};   ///< the reading has not been at rest since

    bool                correcting_{false};
    bool                fast_{false};
    bool                atLimit_{false};
    bool                judging_{false};   ///< the last run's answer is still to be looked at
    bool                probing_{false};   ///< ... and it was a probe
    bool                runTowardsCold_{};
    Temperature         runStart_{};   ///< the reading when the run began
    Position            takeBackTo_{};
    std::optional<bool> lastUp_{};
    Time                settleUntil_{};
    Time                settleMax_{};
    Position            runFrom_{};
    bool                runReversed_{};   ///< the run changed direction: backlash, not the curve
    bool                runClean_{};      ///< water flowed and the reading was at rest
    bool                flowAtRun_{};     ///< water was taken to flow when the run was decided
    std::uint8_t        wrongWay_{};

    Learned       learned_{};
    bool          learnedDirty_{false};
    Time          nextLearnedSave_{FirstLearnedSave};
    Time          nextHotRetry_{};
    Time          nextProbe_{};
    std::uint32_t runs_{};
};

// -- tests ----------------------------------------------------------------------------------------

namespace Test {
    using namespace std::chrono_literals;

    /// The valve on its pipes: Valve::Test::Bench's actuator, boiler and cold water mixing
    /// by the valve's position, a sensor that follows the mix within seconds while water
    /// flows, warms slowly while only the circulation pump runs, and drifts to room
    /// temperature when everything stands.
    ///
    /// The mix was fitted to the real pipes on 2026-09-20 (the one clean run of a night:
    /// 3.76 % of travel for +1.05 K, and 30 % -> 24 % for +1.8 K, so ~0.30 K per % of travel
    /// around a quarter open). That is three times flatter than the ball-valve curve guessed
    /// before it, which is why a gain of 0.8 %/K looked right here and was four times too
    /// small on the pipes. In a straight line is itself a guess outside 20..32 % - that is
    /// the only range there is data for.
    struct Plant {
        Valve::Test::Bench bench;
        Controller         regulator{};
        Target             target{Units::target(600)};
        double             boiler{70.0};
        double             coldWater{16.0};
        double             room{22.0};
        double             bypass{0.12};      ///< the share of cold with the valve closed
        double             maxCold{0.675};    ///< ... and with it wide open
        bool               flow{false};       ///< a tap is drawing
        bool               pump{false};       ///< only the circulation pump: a trickle
        double             pumpLag{1200.0};   ///< the loop warms with this time constant, s
        bool               sensorWorks{true};
        double             sensor{22.0};
        double             cooling{1800.0};    ///< a standing pipe's time constant, s
        bool               coldAtOpen{true};   ///< the way the valve really acts
        /// The water needs this long from the valve to the sensor, and the sensor follows it
        /// with this time constant: measured on the real pipes (2026-09-19: a run showed in
        /// the reading 7 s later, and the reading was at rest 25 s after it).
        static constexpr std::size_t  DeadTurns = 28;   ///< 7 s of bench.dt
        double                        sensorLag{7.0};
        std::array<double, DeadTurns> pipe{};
        std::size_t                   pipeAt{};
        bool                          pipeFilled{false};
        double                        slack{0.0};   ///< backlash, as a share of the travel
        double                        stem{-1.0};   ///< where the valve is behind that backlash
        std::uint32_t                 runs{};
        std::uint32_t                 takeBacks{};
        double                        hottest{};   ///< while water flowed

        constexpr explicit Plant(Position position)
          : bench{Valve::Test::Stored,
                  position} {
            bench.dt = 250ms;
            bench.at = Time{Units::share(bench.openTravel.count(), position, Controller::Full)};
        }

        [[nodiscard]] constexpr double opening() const {
            return static_cast<double>(bench.at.count())
                 / static_cast<double>(bench.openTravel.count());
        }

        [[nodiscard]] constexpr double mix() const {
            auto const o     = coldAtOpen ? stem : 1.0 - stem;
            auto const share = bypass + (maxCold - bypass) * o;
            return boiler - share * (boiler - coldWater);
        }

        constexpr void turn() {
            auto const dt = static_cast<double>(bench.dt.count()) / 1000.0;
            if(stem < 0.0) { stem = opening(); }
            if(opening() > stem + slack / 2) { stem = opening() - slack / 2; }
            if(opening() < stem - slack / 2) { stem = opening() + slack / 2; }
            // what arrives at the sensor left the mix point DeadTurns ago
            if(!pipeFilled) {
                pipe.fill(mix());
                pipeFilled = true;
            }
            auto const arriving = pipe[pipeAt];
            if(flow || pump) {
                pipe[pipeAt] = mix();
                pipeAt       = (pipeAt + 1) % DeadTurns;
            }
            // A draw pulls the mix past the sensor in seconds; the circulation pump only
            // warms the whole loop towards it over half an hour.
            sensor += flow ? (arriving - sensor) * dt / sensorLag
                    : pump ? (arriving - sensor) * dt / pumpLag
                           : (room - sensor) * dt / cooling;
            if(flow && sensor > hottest) { hottest = sensor; }

            auto const& valve = bench.c;
            auto const  out   = regulator.tick(
              {.reading    = sensorWorks ? std::optional{Units::temperature(
                                             static_cast<std::int32_t>(sensor * 1000.0))}
                                         : std::nullopt,
               .target     = target,
               .position   = valve.position(),
               .mode       = valve.mode(),
               .editing    = valve.editing(),
               .calibrated = valve.calibrated()},
              bench.now);
            switch(out.command.kind) {
            case Command::Kind::none: break;
            case Command::Kind::goTo:
                bench.c.goTo(out.command.position);
                ++runs;
                break;
            case Command::Kind::stop: bench.c.stop(bench.now); break;
            case Command::Kind::run:
                bench.c.run(out.command.direction);
                ++runs;
                break;
            }
            for(std::size_t i = 0; i < out.events.count; ++i) {
                if(out.events.list[i].kind == Event::Kind::takeBack) { ++takeBacks; }
            }
            bench.turn();
        }

        constexpr void run(Time d) {
            auto const end = bench.now + d;
            while(bench.now < end) { turn(); }
        }

        [[nodiscard]] constexpr bool near(double degrees,
                                          double by) const {
            return sensor > degrees - by && sensor < degrees + by;
        }
    };

    static_assert(defaults()[std::to_underlying(Parameter::settle)] == 150);
    static_assert(Controller{Stored{.version = Version + 1}}.values() == defaults(),
                  "parameters of another firmware are not used");
    static_assert(
      [] {
          Controller c{};
          bool const changed = c.set(std::to_underlying(Parameter::gain), 30'000, 0ms);
          bool       save    = c.tick({}, 1s).saveParameters;
          save               = save || c.tick({}, 4s).saveParameters;
          return changed && c.get(Parameter::gain) == 1000 && !save && c.tick({}, 6s).saveParameters
              && !c.tick({}, 7s).saveParameters && !c.set(99, 1, 0ms);
      }(),
      "a parameter is clamped to its range and saved once, a while after the change");

    static_assert(
      [] {
          Plant p{Controller::Full / 10};   // where a 65 degC boiler left it
          p.run(10s);                       // standing, at room temperature: nothing
          bool const still = p.runs == 0;
          p.flow           = true;   // and now the boiler has 90
          p.run(15s);
          bool const flowing = p.regulator.status().flowing;
          p.run(120s);
          return still && flowing && p.near(60.0, 4.0) && p.runs <= 8 && !p.bench.switchedUnderLoad;
      }(),
      "a draw with the boiler hotter than last time comes down to the target");

    static_assert(
      [] {
          Plant p{Controller::Full * 12 / 100};   // where the boiler mixes to 60
          p.flow = true;
          p.run(60s);
          return p.runs == 0 && p.near(60.0, 2.0);
      }(),
      "a draw on a cold pipe with the valve where it belongs: the rising reading moves nothing");

    static_assert(
      [] {
          Plant p{Controller::Full * 6 / 10};
          p.boiler = 66.0;   // cooled over night: the valve is too far in the cold
          p.flow   = true;
          p.run(230s);
          return p.near(60.0, 4.5) && p.takeBacks == 0;
      }(),
      "too cold with water flowing goes up to the target");

    static_assert(
      [] {
          Plant p{Controller::Full * 3 / 10};
          p.boiler = 50.0;   // winter: below the target
          p.flow   = true;
          p.run(170s);
          return Units::same(p.bench.c.percent(), Units::percent(0)) && p.regulator.status().atLimit
              && p.bench.c.mode() == Valve::Mode::idle;
      }(),
      "a boiler below the target: the valve ends closed, and that is no fault");

    static_assert(
      [] {
          Plant p{Controller::Full * 21 / 100};
          p.flow = true;
          p.run(60s);
          auto const before = p.bench.c.position();
          p.flow            = false;   // the tap closes, the pipe cools to below the band
          p.cooling         = 400.0;
          auto const runs   = p.runs;
          p.run(110s);
          return p.sensor < 55.0 && p.runs == runs && Units::same(p.bench.c.position(), before)
              && !p.regulator.status().flowing;
      }(),
      "a standing pipe that cools moves nothing");

    static_assert(
      [] {
          Plant p{Controller::Full / 10};
          p.boiler = 90.0;   // a summer boiler: a tenth open is far too hot
          p.flow   = true;
          p.run(25s);       // a short draw, too hot by the time the sensor shows it ...
          p.flow = false;   // ... that ends before the runs against it are answered
          p.run(140s);
          auto const position = p.bench.c.position();
          // colder than it was, which is the safe side, but not at the limit
          return position && *position > Controller::Full / 10
              && *position < Controller::Full * 9 / 10;
      }(),
      "a draw that ends while too hot does not drive the valve to its limit");

    static_assert(
      [] {
          Plant p{Controller::Full / 2};
          p.boiler = 70.0;
          p.sensor = 52.0;   // standing, steady, too cold, and warmer than a standing pipe gets
          p.room   = 52.0;
          // off by default since 2026-09-20 (a night of 31 probes learned nothing); still works
          p.regulator.set(std::to_underlying(Parameter::probePeriod), 600, 0ms);
          p.run(120s);
          auto const percent = p.bench.c.percent();
          return p.takeBacks == 1 && percent && *percent >= Units::percent(49)
              && *percent <= Units::percent(51) && p.runs == 2;
      }(),
      "too cold and steady without a sign of flow: one run to see, taken back");

    static_assert(
      [] {
          Plant p{Controller::Full / 10};
          p.regulator.set(std::to_underlying(Parameter::coldAtOpen), 0, 0ms);
          p.coldAtOpen = false;
          p.bench.at   = p.bench.openTravel * 9 / 10;
          p.bench.c    = Valve::Controller{Valve::Test::Stored, Controller::Full * 9 / 10};
          p.flow       = true;
          p.run(150s);
          return p.near(60.0, 4.0);
      }(),
      "the valve the other way round: coldAtOpen = 0");

    static_assert(
      [] {
          Plant p{Controller::Full * 12 / 100};
          p.flow = true;
          for(double const boiler : {75.0, 90.0, 75.0}) {
              p.boiler = boiler;
              p.run(150s);   // a boiler changes slowly: not while a run is being judged
          }
          // a kelvin is about 3.3 % of this valve's travel (Plant::mix(), fitted 2026-09-20)
          auto const& learned = p.regulator.learned();
          return learned.samples[1] >= Controller::MinSamples && learned.gain[1] >= Units::gain(250)
              && learned.gain[1] <= Units::gain(450) && learned.samples[7] == 0
              && p.near(60.0, 4.0);
      }(),
      "clean runs teach the valve's travel per kelvin, region by region");

    static_assert(
      [] {
          Plant p{Controller::Full * 40 / 100};
          p.regulator.set(std::to_underlying(Parameter::coldAtOpen), 0, 0ms);   // wrong
          p.flow = true;
          p.run(260s);
          return p.regulator.get(Parameter::coldAtOpen) == 1 && p.near(60.0, 4.5);
      }(),
      "a reading that answers the wrong way round turns coldAtOpen over");

    static_assert(
      [] {
          Plant p{Controller::Full * 40 / 100};
          p.regulator.set(std::to_underlying(Parameter::coldAtOpen), 0, 0ms);   // wrong
          p.regulator.set(std::to_underlying(Parameter::learn), 0, 0ms);
          p.flow = true;
          p.run(120s);
          return p.regulator.get(Parameter::coldAtOpen) == 0
              && p.regulator.learned() == Learned{.version = LearnedVersion};
      }(),
      "learn = 0: nothing is learned, nothing is turned over");

    static_assert(
      Controller{
        {},
        Learned{.version = 99,
         .gain    = {StoredGain{Units::gain(500)}}}
    }
          .learned()
          .gain[0]
        == Gain{},
      "what another firmware learned is not used");

    static_assert(
      [] {
          Plant p{Controller::Full / 10};
          p.flow        = true;
          p.sensorWorks = false;
          p.run(30s);
          bool const blind = p.runs == 0 && p.regulator.state() == State::blind;
          p.sensorWorks    = true;
          p.target         = Valve::Off;
          p.run(30s);
          return blind && p.runs == 0 && p.regulator.state() == State::off;
      }(),
      "no reading or no target: the valve stays");

    // -- what a night on the real pipes showed (2026-09-20) --------------------------------------

    /// exp(-y) for y >= 0, by series on the positive exponent so nothing cancels.
    [[nodiscard]] constexpr double decay(double y) {
        double e = 1.0, term = 1.0;
        for(int i = 1; i < 40; ++i) {
            term *= y / i;
            e += term;
        }
        return 1.0 / e;
    }

    /// The draw of 2026-09-20 08:20:47 as the sensor really saw it: at rest at 59.0, the pipe's
    /// stored slug arriving over 6 s to a 63.2 peak, then 90 s of decay to the ~55.1 the valve
    /// was actually mixing. Plant cannot make this - its pipe is 7 s long and its sensor follows
    /// in 7 s, so a draw's arrival is over before the 4 s slope window has a value. The shape is
    /// therefore played back into the Controller directly.
    [[nodiscard]] constexpr Temperature drawOf20260920(Time at) {
        double const t = double(at.count()) / 1000.0;
        double const c = t < 4.0  ? 59.0
                       : t < 10.0 ? 59.0 + (63.2 - 59.0) * (t - 4.0) / 6.0
                                  : 55.1 + (63.2 - 55.1) * decay((t - 10.0) / 30.0);
        return Units::temperature(static_cast<std::int32_t>(c * 1000.0));
    }

    /// The first run of a draw, and which way it went. The valve is taken to reach every goal at
    /// once: what is under test is when the decision is made, not the motor.
    struct FirstRun {
        Time     at{-1};
        Position from{};
        Position to{};
        int      runs{};
    };

    [[nodiscard]] constexpr FirstRun firstRunOfThatDraw(std::int16_t quietMax) {
        Controller c{};
        c.set(std::to_underlying(Parameter::quietMax), quietMax, Time{0});
        FirstRun r{.from = Controller::Full * 30 / 100};
        auto     position = r.from;
        for(Time now{0}; now <= Time{200'000}; now += Time{250}) {
            auto const out = c.tick({.reading    = drawOf20260920(now),
                                     .target     = Units::target(600),
                                     .position   = position,
                                     .mode       = Valve::Mode::idle,
                                     .editing    = false,
                                     .calibrated = true},
                                    now);
            if(out.command.kind == Command::Kind::goTo) {
                ++r.runs;
                if(r.at.count() < 0) {
                    r.at = now;
                    r.to = out.command.position;
                }
                position = out.command.position;
            }
        }
        return r;
    }

    static_assert(
      [] {
          auto const gated
            = firstRunOfThatDraw(defaults()[std::to_underlying(Parameter::quietMax)]);
          // the reading is at rest again, it is too cold by then, and the run goes towards hot
          return gated.at > Time{60'000} && gated.to < gated.from && gated.runs == 1;
      }(),
      "a draw's arriving slug is waited out, and the run that follows goes towards hot");

    static_assert(
      [] {
          auto const ungated = firstRunOfThatDraw(0);
          // what really happened: 5 s into the draw, on a reading still climbing to its peak,
          // 30.1 % -> 32.5 % towards COLD, and the 6 K the slug then lost was credited to it
          return ungated.at < Time{15'000} && ungated.to > ungated.from;
      }(),
      "... and without the gate it runs towards cold on the way up: the bug of 2026-09-20");

    static_assert(
      [] {
          // only the circulation pump: no tap, the loop warming towards the mix over half an hour
          Plant p{Controller::Full * 30 / 100};
          p.sensor = 48.0;
          p.room   = 20.0;
          p.pump   = true;
          p.run(600s);
          return p.regulator.status().flowing && p.sensor > 50.0 && p.sensor < 52.0;
      }(),
      "the circulation pump warming the loop counts as water flowing");

    static_assert(
      [] {
          Plant p{Controller::Full * 30 / 100};
          p.regulator.set(std::to_underlying(Parameter::slowFlow), 0, 0ms);   // as it was
          p.sensor = 48.0;
          p.room   = 20.0;
          p.pump   = true;
          p.run(600s);
          // the whole night of 2026-09-20: the fast slope never leaves the noise, so the valve
          // has authority for hours and the regulator does not know it
          return !p.regulator.status().flowing;
      }(),
      "... and without slowFlow it does not, which is why the pump hours were never used");
}   // namespace Test

}   // namespace Regulator
