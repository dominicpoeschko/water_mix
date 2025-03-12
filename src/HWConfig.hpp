#pragma once

inline bool dbgpres();

struct DbgPres {
    bool operator()() { return dbgpres(); }
};

#include "uc_log/DefaultRttComBackend.hpp"

namespace uc_log {
template<>
struct ComBackend<uc_log::Tag::User>
  : public uc_log::DefaultRttComBackend<DbgPres, 1024, 256, rtt::BufferMode::skip> {};
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

inline bool dbgpres() {
    return true;
    return apply(read(Kvasir::Peripheral::DSU::Registers<>::STATUSB::dbgpres));
}

namespace HW {
using ComBackend = uc_log::ComBackend<uc_log::Tag::User>;

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
        using KSR                              = Kvasir::Peripheral::SYSCTRL::Registers<>;
        using KNR                              = Kvasir::Peripheral::NVMCTRL::Registers<>;
        static constexpr auto flash_waitstates = KNR::CTRLB::RWSVal::half;

        static constexpr auto GENDIV_div = 0;
        apply(KSR::DFLLCTRL::overrideDefaults(
          set(KSR::DFLLCTRL::enable),
          clear(KSR::DFLLCTRL::ondemand)));

        while(!apply(read(KSR::PCLKSR::dfllrdy))) {
        }

        apply(
          write(KSR::DFLLVAL::fine, 0),
          write(KSR::DFLLVAL::diff, 0),
          write(KSR::DFLLVAL::coarse, value<37>()));

        apply(
          KNR::CTRLB::overrideDefaults(
            write(KNR::CTRLB::rws, value<KNR::CTRLB::RWSVal, flash_waitstates>())),
          GenericClockGenerator<0, GeneratorSource::dfll48m, GENDIV_div>::enable());
    }

    static void peripheryClockInit() {
        using Kvasir::Register::value;
        using namespace Kvasir::GCLK;
        apply(
          GenericClockGenerator<1, GeneratorSource::dfll48m, I2CClockDivide>::enable(),
          GenericClockGenerator<2, GeneratorSource::dfll48m, SPIClockDivide>::enable(),
          GenericClockGenerator<3, GeneratorSource::dfll48m, EICClockDivide>::enable(),
          PeripheralChannelController<1, Peripheral::sercom0_core>::enable(),
          PeripheralChannelController<2, Peripheral::sercom3_core>::enable(),
          PeripheralChannelController<3, Peripheral::eic>::enable());
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

struct Fault_CleanUpAction {
    void operator()() {
        apply(clear(Pin::relay1{}, Pin::relay2{}, Pin::relay3{}, Pin::relay4{}, Pin::led{}));
    }
};

struct PinConfig {
    static constexpr auto initStepPinConfig = list(
      makeOutput(HW::Pin::relay1{}),
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
