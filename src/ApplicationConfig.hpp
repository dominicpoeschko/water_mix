#pragma once
#include "HWConfig.hpp"
///

#include "Regulator.hpp"
#include "Valve.hpp"
#include "kvasir/Devices/Max31865.hpp"
#include "kvasir/Util/FaultHandler.hpp"
#include "kvasir/Util/StackProtector.hpp"
#include "kvasir/Util/StackUsage.hpp"
#include "kvasir/Util/Trace.hpp"

using Clock = HW::SystickClock;

struct DmacConfig {
    static constexpr auto numberOfChannels = 2;
};

/// Every event with its time: Valve::Controller tells a press from a bounce by the time
/// between hit and release, and starts calibration on a long press of one button while the
/// other is held.
struct ButtonConfig {
    static constexpr auto longPressTime   = std::chrono::seconds{3};
    static constexpr auto useHit          = true;
    static constexpr auto useShortRelease = true;
    static constexpr auto useLong         = true;
    static constexpr auto useLongRelease  = true;
    static constexpr auto useTime         = true;
    static constexpr auto invert          = true;
    // Measured on this board (ButtonTrace, 2026-09-19): the contact drops out for 0.1 to 8 ms
    // in the middle of a press, and a light tap is over in 12 ms.
    static constexpr auto debouncePress   = std::chrono::milliseconds{3};
    static constexpr auto debounceRelease = std::chrono::milliseconds{25};
};

using Dma = Kvasir::DMAC::DmaBase<DmacConfig>;

/// The queued SERCOM master: the same interface the RP2040 and RP2350 buses present, which
/// is what the kvasir_devices chip descriptions Kvasir::I2C::Device drives are written
/// against. The plain I2CBehavior this used to be does not satisfy them.
using DisplayI2C = Kvasir::Sercom::I2C::I2CBehaviorQueued<HW::I2CConfig, Clock, 8, 16>;
using Spi        = Kvasir::Sercom::SPI::SPIBehavior<HW::SPIConfig,
                                                    Dma,
                                                    Kvasir::DMAC::DMAChannel::ch0,
                                                    Kvasir::DMAC::DMAChannel::ch1,
                                                    Kvasir::DMAC::DMAPriority::p0>;

using Rtd
  = Kvasir::Max31865<Clock, Spi, HW::Pin::cs, HW::Pin::drdy, HW::RtdNominal, HW::RtdReference>;

/// What survives power-off: the two travel times calibration measured. EepromEmulator reads
/// its row of the RWW flash lazily on the first value(); all zero is "none".
using CalibrationEeprom = Kvasir::EepromEmulator<Clock, Valve::Calibration, 2, false>;

/// Where the valve is (Valve::Controller::position()), written once a second while the motor
/// runs: a FlashJournal, which spreads that over 8 rows of the RWW flash with one page write
/// per value - 32 writes per erase cycle of a row, 25k cycles min. (SAM D21 data sheet
/// DS40001882, table 37-43): 800'000 saves at the least. A run of the regulator is a few
/// seconds, so two or three saves; at a hundred runs a day that is a decade. With the
/// calibration's 2 rows, the target's 2, the parameters' 2 and 2 for what the regulator learned
/// that is all 16 of the RWW section.
using PositionJournal = Kvasir::FlashJournal<Clock, Valve::Position, 8, false>;

/// The target temperature, written when it was set with the buttons: the last 2 rows.
using TargetEeprom = Kvasir::EepromEmulator<Clock, Valve::Target, 2, false>;

/// The regulator's parameters, written a few seconds after one was changed over the remote.
using ParameterEeprom = Kvasir::EepromEmulator<Clock, Regulator::Stored, 2, false>;

/// What the regulator has learned about the valve: written once an hour at the most, and
/// only when it changed.
using LearnedEeprom = Kvasir::EepromEmulator<Clock, Regulator::Learned, 2, false>;

using StackProtector = Kvasir::StackProtector<>;
using FaultHandler   = Kvasir::Fault::Handler<HW::Fault_CleanUpAction>;

/// Every button interrupt, as the EIC delivered it: for looking at the contacts' bounce.
/// `kvasir_bench.py trace water_mix/build <variant> button --delta` while the core runs.
/// `button` is a Valve::Button (0 green, 1 red), `level` the pin as read in the interrupt
/// (1: pressed). PushButton debounces nothing: every record here is an event for main().
using ButtonTrace = Kvasir::Trace::Ring<"button", 64, "us", "button", "level">;

template<typename Pin, std::uint32_t Id>
struct TracedButton : Kvasir::SamPushButton<Clock, Pin, 16, ButtonConfig> {
    using Base = Kvasir::SamPushButton<Clock, Pin, 16, ButtonConfig>;

    static void edgeCallback() {
        ButtonTrace::record(
          std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch())
            .count(),
          Id,
          !apply(read(Pin{})));   // ButtonConfig::invert
        Base::edgeCallback();
    }

    struct EicConfig : Base::EicConfig {
        static constexpr auto callback = edgeCallback;
    };
};

using RedButton   = TracedButton<HW::Pin::button2, 1>;   // ids: Valve::Button
using GreenButton = TracedButton<HW::Pin::button1, 0>;

struct EicConfig {
    static constexpr auto IsrPriority = 2;
};

using Eic = Kvasir::EIC::EicBase<Clock, EicConfig, RedButton::EicConfig, GreenButton::EicConfig>;

using Startup = Kvasir::Startup::Startup<
  HW::ClockSettings,
  Clock,
  FaultHandler,
  HW::ComBackend,
  StackProtector,
  Kvasir::StackUsage,
  DisplayI2C,
  Dma,
  Spi,
  Eic,
  // The buttons are not listed: SamPushButton has no init step, Isr or runtime hook of
  // its own - its pin and edge interrupt come from RedButton/GreenButton::EicConfig,
  // which Eic above already carries, and main() drives their handler() itself.
  HW::PinConfig>;
