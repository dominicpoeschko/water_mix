#pragma once

inline bool dbgpres();

struct DbgPres {
    bool operator()() { return dbgpres(); }
};

#include "uc_log/DefaultRttComBackend.hpp"

namespace uc_log {
// A record from an ISR is written with interrupts masked (IsrPolicy::MaskedRecord), so ISRs at
// different priority levels share the ISR buffer; Startup refuses that mix of levels otherwise.
template<>
struct ComBackend<uc_log::Tag::User>
  : public uc_log::DefaultRttComBackend<DbgPres,
                                        1024,
                                        256,
                                        rtt::BufferMode::skip,
                                        // "control": Protocol.hpp, see Remote.hpp. uc_log_printer
                                        // puts it on a socket, duplex.0.sock, for tui/.
                                        DuplexChannels<DuplexChannelConfig<"control", 256, 128>>,
                                        IsrPolicy::MaskedRecord> {};
}   // namespace uc_log

// need to be included first

#include "chip/Interrupt.hpp"
#include "core/core.hpp"
#include "uc_log/uc_log.hpp"

namespace HW {
static constexpr auto ClockSpeed     = 48'000'000;
static constexpr auto I2CClockDivide = 32;
static constexpr auto EICClockDivide = 4096;
static constexpr auto SPIClockDivide = 2;
static constexpr auto WDTClockDivide = 32;   // OSCULP32K down to the 1.024 kHz the WDT counts

struct SystickClockConfig {
    static constexpr auto clockBase = Kvasir::Systick::useProcessorClock;

    static constexpr auto clockSpeed     = ClockSpeed;
    static constexpr auto minOverrunTime = std::chrono::years(20);
};

using SystickClock = Kvasir::Systick::SystickClockBase<SystickClockConfig>;
}   // namespace HW

namespace uc_log {
template<>
struct LogClock<uc_log::Tag::User> : public HW::SystickClock {};
}   // namespace uc_log

#include "chip/chip.hpp"
#include "kvasir/Devices/Quantities.hpp"

/// Logging goes out over RTT only while a debugger is attached.
inline bool dbgpres() {
    return apply(read(Kvasir::Peripheral::DSU::Registers<>::STATUSB::dbgpres));
}

namespace HW {
using ComBackend = uc_log::ComBackend<uc_log::Tag::User>;

/// The RTD on the MAX31865 and the reference resistor next to it.
static constexpr auto RtdNominal   = Kvasir::Units::ohm(500);
static constexpr auto RtdReference = Kvasir::Units::ohm(1000);

namespace Pin {
    using ac1 = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin2));
    using ac2 = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin3));
    using ac3 = decltype(makePinLocation(Kvasir::Io::portB, Kvasir::Io::pin4));
    using ac4 = decltype(makePinLocation(Kvasir::Io::portB, Kvasir::Io::pin5));

    using relay1 = decltype(makePinLocation(Kvasir::Io::portB, Kvasir::Io::pin0));
    using relay2 = decltype(makePinLocation(Kvasir::Io::portB, Kvasir::Io::pin1));
    using relay3 = decltype(makePinLocation(Kvasir::Io::portB, Kvasir::Io::pin2));
    using relay4 = decltype(makePinLocation(Kvasir::Io::portB, Kvasir::Io::pin3));

    using led = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin16));

    using drdy = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin21));
    using sdo  = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin22));
    using sclk = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin23));
    using cs   = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin24));
    using sdi  = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin25));

    using sda = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin4));
    using scl = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin5));

    using button1 = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin6));
    using button2 = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin7));
}   // namespace Pin

struct ClockSettings {
    static void coreClockInit() {
        using Kvasir::Register::value;

        using namespace Kvasir::GCLK;
        using KNR                              = Kvasir::Peripheral::NVMCTRL::Registers<>;
        static constexpr auto flash_waitstates = KNR::CTRLB::RWSVal::half;

        static constexpr auto GENDIV_div = 0;

        // 48 MHz from the DFLL in open loop, on the coarse value the factory measured for
        // this part.
        Kvasir::DFLL::enableOpenLoop();

        apply(KNR::CTRLB::overrideDefaults(
                write(KNR::CTRLB::rws, value<KNR::CTRLB::RWSVal, flash_waitstates>())),
              GenericClockGenerator<0, GeneratorSource::dfll48m, GENDIV_div>::enable());
    }

    static void peripheryClockInit() {
        using Kvasir::Register::value;
        using namespace Kvasir::GCLK;
        apply(GenericClockGenerator<1, GeneratorSource::dfll48m, I2CClockDivide>::enable(),
              GenericClockGenerator<2, GeneratorSource::dfll48m, SPIClockDivide>::enable(),
              GenericClockGenerator<3, GeneratorSource::dfll48m, EICClockDivide>::enable(),
              GenericClockGenerator<4, GeneratorSource::osculp32k, WDTClockDivide>::enable(),
              PeripheralChannelController<1, Peripheral::sercom0_core>::enable(),
              PeripheralChannelController<2, Peripheral::sercom3_core>::enable(),
              PeripheralChannelController<3, Peripheral::eic>::enable(),
              PeripheralChannelController<4, Peripheral::wdt>::enable());
    }
};

struct I2CConfig {
    static constexpr auto clockSpeed = ClockSpeed / I2CClockDivide;

    static constexpr auto instance       = 0;
    static constexpr auto sdaPinLocation = Pin::sda{};
    static constexpr auto sclPinLocation = Pin::scl{};
    static constexpr auto baudRate       = 100'000;
    static constexpr auto isrPriority    = 1;
};

struct SPIConfig {
    static constexpr auto clockSpeed = ClockSpeed / SPIClockDivide;

    static constexpr auto instance        = 3;
    static constexpr auto misoPinLocation = Pin::sdo{};
    static constexpr auto mosiPinLocation = Pin::sdi{};
    static constexpr auto sclkPinLocation = Pin::sclk{};
    static constexpr auto csPinLocation   = Kvasir::Sercom::SPI::NotUsed<>{};
    static constexpr auto baudRate        = 1'000'000;
    static constexpr auto mode            = Kvasir::Sercom::SPI::Mode::_3;
    static constexpr auto isrPriority     = 1;
};

/// The watchdog: CONFIG.PER 0x8 is 2048 cycles of GCLK_WDT (datasheet DS40001882, WDT
/// CONFIG), which generator 4 makes 32.768 kHz / 32 = 1.024 kHz of the OSCULP32K, so 2 s --
/// give or take that oscillator's accuracy. A main loop that stops turning lets go of the
/// relays through the reset (the pins are inputs again). "When the CPU is halted in debug
/// mode the WDT will halt normal operation" (WDT, Debug Operation). Not always-on, so a
/// reset -- flashing included -- starts without it.
struct Watchdog {
    using Regs = Kvasir::Peripheral::WDT::Registers<>;

    static void sync() {
        while(apply(read(Regs::STATUS::syncbusy))) {}
    }

    static void init() {
        apply(write(Regs::CONFIG::PERValC::_2k));
        sync();
        apply(set(Regs::CTRLA::enable));
        sync();
    }

    /// From the main loop only. A clear while the last one still synchronises is dropped:
    /// the next turn brings another.
    static void handler() {
        if(!apply(read(Regs::STATUS::syncbusy))) { apply(write(Regs::CLEAR::CLEARValC::key)); }
    }
};

struct Fault_CleanUpAction {
    void operator()() {
        apply(clear(Pin::relay1{}, Pin::relay2{}, Pin::relay3{}, Pin::relay4{}, Pin::led{}));
    }
};

struct PinConfig {
    static constexpr auto initStepPinConfig = list(makeOutput(HW::Pin::relay1{}),
                                                   makeOutput(HW::Pin::relay2{}),
                                                   makeOutput(HW::Pin::relay3{}),
                                                   makeOutput(HW::Pin::relay4{}),
                                                   makeOutput(HW::Pin::led{}),
                                                   makeInput(HW::Pin::ac1{}),
                                                   makeInput(HW::Pin::ac2{}),
                                                   makeInput(HW::Pin::ac3{}),
                                                   makeInput(HW::Pin::ac4{}),
                                                   set(HW::Pin::led{}));
};

}   // namespace HW
