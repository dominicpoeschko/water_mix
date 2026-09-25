#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mp-units/systems/si.h>
#include <optional>
#include <string_view>

/// The physical values of the valve and its regulator as mp-units quantities: the scale and
/// the dimension are in the type, so a temperature is not added to a position and tenths are
/// not taken for thousandths. Nothing but the standard library and mp-units in here: the
/// firmware and the tui both include it.
///
/// All integral - no soft float on the Cortex-M0+ - and each the size of the integer it holds,
/// so what goes to flash and over the wire has the layout the plain integers had.
///
/// Time is std::chrono, which is the same idea and what every clock speaks; where a time
/// meets a quantity (a rate) it goes through seconds().
namespace Units {

namespace mpu = mp_units;
namespace si  = mp_units::si;

// -- temperature ------------------------------------------------------------------------------
//
// Celsius is an offset unit: a value is built with delta<> - the distance from the ice point,
// which is what the RTD driver reports (Kvasir::Units::MilliDegC is this very type).
inline constexpr auto milliDegC = si::milli<si::degree_Celsius>;
inline constexpr auto deciDegC  = si::deci<si::degree_Celsius>;
inline constexpr auto milliK    = si::milli<si::kelvin>;
inline constexpr auto deciK     = si::deci<si::kelvin>;

/// A reading.
using Temperature = mpu::quantity<milliDegC, std::int32_t>;
/// How far two temperatures are apart: an error, a band, what a run changed.
using Kelvins = mpu::quantity<milliK, std::int32_t>;
/// A temperature to set, in the steps the display has. As a target, zero is "off". int16:
/// the flash format.
using Target = mpu::quantity<deciDegC, std::int16_t>;
/// A temperature as the three digits show it.
using Shown = mpu::quantity<deciDegC, std::int32_t>;
/// How fast a reading moves.
using Rate = mpu::quantity<milliK / si::second, std::int32_t>;

// -- the valve's travel -----------------------------------------------------------------------

inline constexpr auto milliPercent = si::milli<mpu::percent>;
inline constexpr auto deciPercent  = si::deci<mpu::percent>;

/// Where the valve is, and how far a run goes: 0 closed, 100 % open.
using Position = mpu::quantity<milliPercent, std::int32_t>;
/// The same for a display.
using Percent = mpu::quantity<mpu::percent, std::uint8_t>;
/// Travel per kelvin: the valve's curve.
using Gain = mpu::quantity<si::centi<mpu::percent> / si::kelvin, std::int32_t>;

inline constexpr Position Closed = 0 * milliPercent;
inline constexpr Position Full   = 100'000 * milliPercent;

// -- building them ----------------------------------------------------------------------------

[[nodiscard]] constexpr Temperature temperature(std::int32_t milli) {
    return mpu::delta<milliDegC>(milli);
}

[[nodiscard]] constexpr Kelvins kelvins(std::int32_t milli) { return mpu::delta<milliK>(milli); }

[[nodiscard]] constexpr Kelvins deciKelvins(std::int32_t tenths) {
    return mpu::delta<deciK>(tenths);
}

[[nodiscard]] constexpr Target target(std::int16_t tenths) { return mpu::delta<deciDegC>(tenths); }

[[nodiscard]] constexpr Rate rate(std::int32_t milliKelvinPerSecond) {
    return mpu::delta<milliK>(milliKelvinPerSecond) / (1 * si::second);
}

[[nodiscard]] constexpr Position position(std::int32_t milli) { return milli * milliPercent; }

[[nodiscard]] constexpr Position deciPercents(std::int32_t tenths) { return tenths * deciPercent; }

[[nodiscard]] constexpr Percent percent(std::uint8_t whole) { return whole * mpu::percent; }

[[nodiscard]] constexpr Gain gain(std::int32_t centiPercentPerKelvin) {
    return centiPercentPerKelvin * (si::centi<mpu::percent> / si::kelvin);
}

/// A time as a quantity, for where it meets one.
template<typename Rep,
         typename Period>
[[nodiscard]] constexpr auto milliseconds(std::chrono::duration<Rep,
                                                                Period> d) {
    return static_cast<std::int32_t>(
             std::chrono::duration_cast<std::chrono::milliseconds>(d).count())
         * si::milli<si::second>;
}

// -- taking them apart ------------------------------------------------------------------------

/// The integer a quantity holds, in its own unit: for the display's digits, an index, a
/// parameter table - where the number leaves the type system.
template<auto R,
         typename Rep>
[[nodiscard]] constexpr Rep raw(mpu::quantity<R,
                                              Rep> q) {
    return q.numerical_value_in(q.unit);
}

template<auto R,
         typename Rep>
[[nodiscard]] constexpr mpu::quantity<R,
                                      Rep>
magnitude(mpu::quantity<R,
                        Rep> q) {
    return q < q.zero() ? -q : q;
}

template<auto R,
         typename Rep>
[[nodiscard]] constexpr mpu::quantity<R,
                                      Rep>
clamp(mpu::quantity<R,
                    Rep> q,
      mpu::quantity<R,
                    Rep> low,
      mpu::quantity<R,
                    Rep> high) {
    return q < low ? low : (q > high ? high : q);
}

/// `a == b` for optionals of a quantity. Not spelled ==: with libc++ that does not compile -
/// mp-units' operator== asks whether the optional is a number to compare the quantity with,
/// which asks whether the optional is equality comparable, which asks mp-units' operator==.
template<typename Q>
[[nodiscard]] constexpr bool same(std::optional<Q> const& a,
                                  std::optional<Q> const& b) {
    return a.has_value() == b.has_value() && (!a || *a == *b);
}

template<typename Q>
[[nodiscard]] constexpr bool same(std::optional<Q> const& a,
                                  Q const&                b) {
    return a && *a == b;
}

/// Rounded to the display's tenth of a degree.
[[nodiscard]] constexpr Shown shown(Temperature t) {
    return mpu::value_cast<deciDegC>(t + temperature(t < t.zero() ? -50 : 50));
}

/// Rounded to the display's whole percent.
[[nodiscard]] constexpr Percent wholePercent(Position p) {
    return mpu::value_cast<std::uint8_t>(mpu::value_cast<mpu::percent>(p + position(500)));
}

/// How far `part` is of `whole`, applied to `of`: the rule of three in 64 bits, where the
/// product does not fit 32.
template<typename T>
[[nodiscard]] constexpr T share(T        of,
                                Position part,
                                Position whole) {
    return static_cast<T>(static_cast<std::int64_t>(of) * raw(part) / raw(whole));
}

/// A unit's symbol in plain ASCII ("dK", "d%", "mK/s", "ddegC"): mp-units' portable symbols,
/// with "deg" for the backtick it writes a degree sign as. For a table column, and for a string
/// that goes over the wire as it is.
namespace detail {
    template<auto U>
    inline constexpr auto symbolStorage = [] {
        static constexpr auto sym
          = mpu::unit_symbol<mpu::unit_symbol_formatting{.char_set = mpu::character_set::portable}>(
            U);
        constexpr std::size_t degrees = [] {
            std::size_t n = 0;
            for(std::size_t k = 0; k < sym.size(); ++k) { n += sym[k] == '`' ? 1 : 0; }
            return n;
        }();
        std::array<char, sym.size() + 2 * degrees> out{};
        std::size_t                                i = 0;
        for(std::size_t k = 0; k < sym.size(); ++k) {
            if(sym[k] == '`') {
                for(char c : std::string_view{"deg"}) { out[i++] = c; }
            } else {
                out[i++] = sym[k];
            }
        }
        return out;
    }();
}   // namespace detail

template<auto U>
[[nodiscard]] constexpr std::string_view symbol() {
    return {detail::symbolStorage<U>.data(), detail::symbolStorage<U>.size()};
}

namespace literals {
    /// 38.5_degC: a Target.
    [[nodiscard]] constexpr Target operator""_degC(long double degrees) {
        return target(static_cast<std::int16_t>(degrees * 10.0L + (degrees < 0 ? -0.5L : 0.5L)));
    }

    [[nodiscard]] constexpr Target operator""_degC(unsigned long long degrees) {
        return target(static_cast<std::int16_t>(degrees * 10));
    }

    /// 30_pct: a whole Percent.
    [[nodiscard]] constexpr Percent operator""_pct(unsigned long long whole) {
        return percent(static_cast<std::uint8_t>(whole));
    }
}   // namespace literals

static_assert(sizeof(Target) == sizeof(std::int16_t) && sizeof(Position) == sizeof(std::int32_t)
                && sizeof(Temperature) == sizeof(std::int32_t),
              "a quantity is the integer it holds: the flash and wire formats rest on it");
static_assert(temperature(61'000) - target(600) == kelvins(1'000));
static_assert(mpu::value_cast<deciK>(kelvins(1'090)) * gain(80) == position(800),
              "an error times the travel per kelvin is a run");
static_assert(wholePercent(position(49'500)) == percent(50) && wholePercent(Full) == percent(100));
static_assert(shown(temperature(38'449)) == Shown{target(384)}
              && shown(temperature(-1'250)) == Shown{target(-13)});
static_assert(share(20'000,
                    Full / 4,
                    Full)
              == 5'000);
static_assert(symbol<deciDegC>() == "ddegC" && symbol<Rate::unit>() == "mK/s"
              && symbol<Gain::unit>() == "c%/K");

}   // namespace Units
