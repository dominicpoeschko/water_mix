#include "ApplicationConfig.hpp"
// need to be included first
#include "cmake_git_version/version.hpp"
#include "displayManager.hpp"
#include "kvasir/Devices/Max31865.hpp"
#include "kvasir/Devices/pca9956b.hpp"
#include "uc_log/uc_log.hpp"

int main() {
    UC_LOG_D("{} reset cause: {}", CMakeGitVersion::FullVersion,
             Kvasir::PM::reset_cause());

    Kvasir::Max31865<Clock, Spi, HW::Pin::cs, HW::Pin::drdy> max31865;

    auto display = DisplayManager<DisplayI2C, Clock>{};

    auto close = [&]() {
        display.setBlinkMode();
        apply(set(HW::Pin::relay1{}));
        apply(clear(HW::Pin::relay2{}));
    };
    auto open = [&]() {
        display.setBlinkMode();
        apply(set(HW::Pin::relay1{}));
        apply(set(HW::Pin::relay2{}));
    };

    auto off = [&]() {
        display.setStaticMode();
        apply(clear(HW::Pin::relay1{}));
        apply(clear(HW::Pin::relay2{}));
    };

    auto next = Clock::time_point{} + 1s;

    enum class State { Idle, Open, Close };

    State state = State::Idle;

    while (true) {
        auto const currentTime = Clock::now();

        if (currentTime > next) {
            std::uint16_t const temperature =
                static_cast<std::uint16_t>(max31865.t().value_or(0) * 10.0f);
            display.set(temperature, 1);

            next += 100ms;
        }

        RedButton::handler([&](auto event) {
            UC_LOG_D("red {}", event);
            switch (event) {
                case RedButton::Event::Type::Release_Short: {
                    if (state != State::Idle) {
                        state = State::Idle;
                    } else {
                        state = State::Close;
                    }
                } break;
            }
        });

        GreenButton::handler([&](auto event) {
            UC_LOG_D("green {}", event);
            switch (event) {
                case GreenButton::Event::Type::Release_Short: {
                    if (state != State::Idle) {
                        state = State::Idle;
                    } else {
                        state = State::Open;
                    }
                } break;
            }
        });

        auto const limitSwitches = apply(read(HW::Pin::ac1{}, HW::Pin::ac2{}));
        switch (state) {
            case State::Idle: {
                off();
            } break;
            case State::Open: {
                if (get<0>(limitSwitches)) {
                    UC_LOG_D("limit 1");
                    state = State::Idle;
                }
                open();
            } break;
            case State::Close: {
                if (get<1>(limitSwitches)) {
                    UC_LOG_D("limit 2");
                    state = State::Idle;
                }
                close();
            } break;
        }

        display.handler();
        max31865.handler();
        StackProtector::handler();
    }
}

KVASIR_START(Startup)
