#pragma once
// Shared by the firmware (Remote.hpp) and the host (tui/): what goes over uc_log's "control"
// duplex channel. Nothing but the standard library, Valve.hpp, Regulator.hpp and aglio in here, so that both
// sides can include it. The messages are aglio-serialised variants in aglio::Packager frames.

#include "Regulator.hpp"
#include "Valve.hpp"

#include <aglio/packager.hpp>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <variant>

// A quantity goes over the wire as the number it holds, like a std::chrono::duration: both
// sides have the unit in the type.
namespace aglio {
template<auto R, typename Rep, typename Size_t>
struct serializer<mp_units::quantity<R, Rep>, Size_t> {
    template<typename Buffer>
    static constexpr bool serialize(mp_units::quantity<R,
                                                       Rep> const& v,
                                    Buffer&                        buffer) {
        return serializer<Rep, Size_t>::serialize(v.numerical_value_in(v.unit), buffer);
    }

    template<typename Buffer>
    static constexpr bool deserialize(mp_units::quantity<R,
                                                         Rep>& v,
                                      Buffer&                  buffer) {
        Rep number;
        if(!serializer<Rep, Size_t>::deserialize(number, buffer)) { return false; }
        v.numerical_value_ref_in(v.unit) = number;
        return true;
    }
};

template<auto R, typename Rep, typename Size_t>
    requires has_fixed_serialized_size<Rep, Size_t>
struct serialized_size<mp_units::quantity<R, Rep>, Size_t> : serialized_size<Rep, Size_t> {};
}   // namespace aglio

namespace Protocol {

// ---- host -> valve -----------------------------------------------------------------------

/// A short press of one button: show the target temperature, step it up (red) or down
/// (green), stop a run, acknowledge a fault - whatever the button on the board does right now.
struct Press {
    Valve::Button button{};
};

/// The motor by hand, which the buttons on the board cannot do (held, they step the
/// target): from idle, runs it open (green) or close (red) to the limit or to the next Press.
struct Hold {
    Valve::Button button{};
};

/// The target temperature at once, instead of stepping to it; Valve::Off: off. One that the
/// buttons could not reach is not taken.
struct SetTarget {
    Valve::Target target{};
};

/// Both buttons held and one of them long-pressed: starts calibration (from idle only).
struct Calibrate {};

/// Asks for a State.
struct Get {};

/// One of the regulator's parameters (Regulator::Parameter, Regulator::Info has names and
/// ranges); clamped to its range, in use at once, in flash a few seconds later.
struct SetParameter {
    std::uint8_t index{};
    std::int16_t value{};
};

/// All parameters back to Regulator::defaults().
struct ResetParameters {};

/// Asks for a Parameters and a Learned.
struct GetParameters {};

/// Forgets what the regulator has learned about the valve.
struct ResetLearned {};

/// The motor to a position, 0 % closed .. 100 % open: from idle and with a known position
/// only. With a target set the regulator takes over again afterwards.
struct GoTo {
    Valve::Position position{};
};

/// Motor off, now: ends a run (by hand, GoTo, the regulator's) and a calibration.
struct Stop {};

using Command = std::variant<Press,
                             Calibrate,
                             Get,
                             Hold,
                             SetTarget,
                             SetParameter,
                             ResetParameters,
                             GetParameters,
                             ResetLearned,
                             GoTo,
                             Stop>;

/// A value that may not be known. Not std::optional: State is compared with ==, and an
/// optional of a quantity cannot be (Units::same() says why).
template<typename Q>
struct Maybe {
    bool known{};
    Q    value{};

    // an aggregate, which is what aglio can take apart: so of() and no constructor
    [[nodiscard]] static constexpr Maybe of(std::optional<Q> const& from) {
        return {.known = from.has_value(), .value = from.value_or(Q{})};
    }

    [[nodiscard]] constexpr std::optional<Q> get() const {
        return known ? std::optional{value} : std::nullopt;
    }

    [[nodiscard]] constexpr bool operator==(Maybe const&) const = default;
};

// ---- valve -> host -----------------------------------------------------------------------

/// Sent when asked for, and whenever anything in it changes (the position and the
/// temperature a few times a second at most).
struct State {
    Valve::Mode               mode{};
    Valve::Fault              fault{};
    bool                      power{};       ///< the power relay
    bool                      direction{};   ///< the direction relay; set: open
    bool                      limitOpen{};   ///< the limit inputs as read, unfiltered
    bool                      limitClose{};
    bool                      calibrated{};
    Valve::Calibration        calibration{};   ///< the one in use
    Maybe<Valve::Position>    position{};      ///< not known: no calibration, no limit seen
    Maybe<Units::Temperature> temperature{};   ///< not known: no reading
    std::uint8_t              rtdFault{};    ///< the MAX31865's fault status register, as last read
    std::uint32_t             rtdFaults{};   ///< conversions rejected for the fault flag since boot
    Valve::View::Kind         view{};        ///< what the display shows
    bool                      blink{};
    Valve::Target             target{};        ///< the target temperature in use; Valve::Off: off
    Valve::Target             shownTarget{};   ///< view == target: the one being set

    [[nodiscard]] constexpr bool operator==(State const&) const = default;
};

/// What the regulator is doing: sent when asked for a State, and when it changes (the
/// numbers a few times a second at most).
struct Regulation {
    Regulator::Status status{};

    [[nodiscard]] constexpr bool operator==(Regulation const&) const = default;
};

/// The regulator's parameters, in the order of Regulator::Parameter: sent when asked for and
/// when one changed.
struct Parameters {
    Regulator::Values values{};
};

/// What the regulator has learned about the valve: sent when asked for the parameters and
/// when it changes.
struct Learned {
    Regulator::Learned learned{};
};

using Report = std::variant<State, Regulation, Parameters, Learned>;

// ---- framing -------------------------------------------------------------------------------

/// CRC-16/CCITT-FALSE, in software on both sides so that they cannot disagree.
struct Crc {
    using type = std::uint16_t;

    static constexpr type calc(std::span<std::byte const> data) {
        std::uint32_t crc = 0xFFFF;
        for(auto const byte : data) {
            crc ^= static_cast<std::uint32_t>(byte) << 8U;
            for(int bit = 0; bit < 8; ++bit) {
                crc = ((crc & 0x8000U) != 0 ? (crc << 1U) ^ 0x1021U : crc << 1U) & 0xFFFFU;
            }
        }
        return static_cast<type>(crc);
    }
};

struct PackagerConfig {
    using Crc                                   = Protocol::Crc;
    using Size_t                                = std::uint8_t;
    static constexpr std::uint16_t PackageStart = 0x55AA;
    // the Parameters report is the longest: 63 bytes as a frame with 27 parameters
    static constexpr Size_t MaxSize = 96;
};

using Packager = aglio::Packager<PackagerConfig>;

}   // namespace Protocol
