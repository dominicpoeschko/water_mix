#pragma once

#include "Units.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/SegmentBackends.hpp>
#include <kvasir/Devices/I2C/chips/Pca9956b.hpp>
#include <optional>
#include <string_view>

/// The three digits on the PCA9956B: the temperature in tenths, the valve's position in
/// percent or a word, steady or blinking. The glyphs, the wiring and the wire traffic are Kvasir::I2C::Pca9956bDisplay's.
template<typename I2C, typename Clock>
struct DisplayManager {
    /// 0x3F with a 2.2 k external resistor, as the board is built. Address and Rext used
    /// to be constructor arguments (`display(0x3F, 2200)`); they are template parameters
    /// of the chip description now, so the IREF arithmetic happens at compile time.
    using Leds   = Kvasir::I2C::Chips::Pca9956b<0x3F, Kvasir::Units::ohm(2200)>;
    using Device = Kvasir::I2C::Device<I2C, Clock, Leds>;

    Device                               display{};
    Kvasir::I2C::Pca9956bDisplay<Device> digits{display, Kvasir::Units::milliAmp(5)};

    DisplayManager()                                 = default;
    DisplayManager(DisplayManager const&)            = delete;   // digits points at display
    DisplayManager& operator=(DisplayManager const&) = delete;

    void blink(bool on) {
        digits.blink(on ? Kvasir::SegmentDisplay::Blink::on : Kvasir::SegmentDisplay::Blink::off);
    }

    /// Tenths of a degree, -9.9 to 99.9 and clamped to that; without a reading, dashes.
    void showTemperature(std::optional<Units::Shown> temperature) {
        if(temperature) {
            digits.setNumber(Units::raw(*temperature), 1, Kvasir::SegmentDisplay::Overflow::clamp);
        } else {
            digits.dashes();
        }
    }

    /// 0 to 100, without a point: what tells it from a temperature.
    void showPercent(Units::Percent percent) { digits.setNumber(Units::raw(percent)); }

    /// Three characters SegmentDisplay::glyph() has: "CAL", "Err".
    void showText(std::string_view text) {
        std::array<Kvasir::SegmentDisplay::Glyph, 3> glyphs{};
        for(std::size_t i = 0; i < glyphs.size() && i < text.size(); ++i) {
            glyphs[i] = Kvasir::SegmentDisplay::glyph(text[i]);
        }
        digits.setDigits(glyphs);
    }

    void handler() {
        digits.update(Clock::now());
        display.handler();
    }
};
