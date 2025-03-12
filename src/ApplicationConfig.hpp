#pragma once
#include "HWConfig.hpp"
///

#include "kvasir/Util/FaultHandler.hpp"
#include "kvasir/Util/StackProtector.hpp"

using Clock = HW::SystickClock;

struct DmacConfig {
    static constexpr auto numberOfChannels = 2;
};

struct ButtonConfig {
    static constexpr auto useShortRelease = true;
    static constexpr auto invert = true;
};

using Dma = Kvasir::DMAC::DmaBase<DmacConfig>;

using DisplayI2C = Kvasir::Sercom::I2C::I2CBehavior<HW::I2CConfig, Clock, 32>;
using Spi        = Kvasir::Sercom::SPI::SPIBehavior<
  HW::SPIConfig,
  Dma,
  Kvasir::DMAC::DMAChannel::ch0,
  Kvasir::DMAC::DMAChannel::ch1,
  Kvasir::DMAC::DMAPriority::p0>;

using StackProtector = Kvasir::StackProtector<>;
using FaultHandler   = Kvasir::Fault::Handler<HW::Fault_CleanUpAction>;

using RedButton   = Kvasir::SamPushButton<Clock, HW::Pin::button2, 64, ButtonConfig>;
using GreenButton = Kvasir::SamPushButton<Clock, HW::Pin::button1, 64, ButtonConfig>;

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
  DisplayI2C,
  Dma,
  Spi,
  Eic,
  RedButton,
  GreenButton,
  HW::PinConfig>;
