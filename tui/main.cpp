// The valve's remote control: connects to the socket uc_log_printer opens for the firmware's
// "control" duplex channel and speaks Protocol.hpp over it. By default that is the unix domain
// socket build/rtt_log/<target>/duplex.0.sock of whichever `just log` runs in this tree;
// a printer started with --transport tcp is reached by host and port.
//
//   water_mix_tui                            the running printer's unix socket
//   water_mix_tui <path/to/duplex.0.sock>
//   water_mix_tui <host> [port]              TCP, port 34610 unless given
//   water_mix_tui --dump [green|red|hold-green|hold-red|calibrate|target=DEG|target=off ...]
//                        [goto=PERCENT] [stop] [NAME=VALUE ...] [defaults] [forget]
//                        [seconds=N] [socket=PATH | port=N]
//                        NAME: one of the regulator's parameters (Regulator::Info), e.g. gain=120
//
// The regulator's parameters: up / down picks one, left / right (or - / +) changes it by one,
// < / > by ten, d sets the picked one to its default, D D all of them. They are in use at
// once and in the valve's flash five seconds later. Below them what the regulator has learned
// about the valve (travel per kelvin by region of the travel, backlash); L L forgets it.
//
// p opens the position entry: digits (a percent, 0 .. 100, one decimal), Enter runs the motor
// there, Esc cancels. s or the space bar stops the motor at once, whatever runs it.
//
// g / r are short presses (the target temperature: show, green down, red up; stop a run;
// acknowledge a fault), G / R the MOTOR by hand (open / close; not possible on the board's
// buttons), c c both held long (calibration).
//
// A command becomes button events in Valve::Controller: what the two buttons on the board do,
// and the motor by hand (G / R), which they cannot. The board is the truth: everything shown is the State it reports.

#include "Protocol.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <ftxui/component/component.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>
#include <functional>
#include <mutex>
#include <netdb.h>
#include <optional>
#include <string>
#include <string_view>
#include <sys/socket.h>
#include <sys/un.h>
#include <system_error>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

namespace {

using namespace std::chrono_literals;

/// Where this tree's printers put their sockets (tui/CMakeLists.txt).
constexpr char const* RttLogDir = TUI_RTT_LOG_DIR;

constexpr char const* TcpHost = "127.0.0.1";
constexpr char const* TcpPort = "34610";   // water_mix/justfile duplex_base_port

std::string whereText(std::string const& host,
                      std::string const& port) {
    if(host.empty()) { return std::string{RttLogDir} + "/*/duplex.0.sock"; }
    return host.find('/') != std::string::npos ? host : host + ":" + port;
}

/// The link to the valve: reconnects on its own, hands every Report to `onReport`.
class Link {
public:
    Link(std::string                                  host,
         std::string                                  port,
         std::function<void(Protocol::Report const&)> onReport,
         std::function<void()>                        onChange)
      : host_{std::move(host)}
      , port_{std::move(port)}
      , onReport_{std::move(onReport)}
      , onChange_{std::move(onChange)}
      , thread_{[this] { run(); }} {}

    ~Link() {
        stop_ = true;
        closeSocket();
        thread_.join();
    }

    [[nodiscard]] bool connected() const { return connected_; }

    void send(Protocol::Command const& command) {
        std::vector<std::byte> frame;
        if(!Protocol::Packager::pack(frame, command)) { return; }
        std::lock_guard const lock{mutex_};
        if(fd_ >= 0) { static_cast<void>(::send(fd_, frame.data(), frame.size(), MSG_NOSIGNAL)); }
    }

private:
    void closeSocket() {
        std::lock_guard const lock{mutex_};
        if(fd_ >= 0) {
            ::shutdown(fd_, SHUT_RDWR);
            ::close(fd_);
            fd_ = -1;
        }
    }

    /// `host_` with a '/' in it is a unix socket's path; empty: whichever printer of this
    /// build tree serves the "control" channel (build/rtt_log/<target>/duplex.0.sock).
    [[nodiscard]] static int openUnix(std::string const& path) {
        sockaddr_un address{};
        address.sun_family = AF_UNIX;
        if(path.size() >= sizeof(address.sun_path)) { return -1; }
        std::copy(path.begin(), path.end(), address.sun_path);
        int const fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if(fd < 0) { return -1; }
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): the sockets API
        if(::connect(fd, reinterpret_cast<sockaddr const*>(&address), sizeof(address)) == 0) {
            return fd;
        }
        ::close(fd);
        return -1;
    }

    [[nodiscard]] int open() const {
        if(host_.empty()) {
            std::error_code ec;
            for(auto const& target : std::filesystem::directory_iterator{RttLogDir, ec}) {
                auto const fd = openUnix((target.path() / "duplex.0.sock").string());
                if(fd >= 0) { return fd; }
            }
            return -1;
        }
        if(host_.find('/') != std::string::npos) { return openUnix(host_); }

        addrinfo hints{};
        hints.ai_family   = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        addrinfo* list    = nullptr;
        if(::getaddrinfo(host_.c_str(), port_.c_str(), &hints, &list) != 0) { return -1; }
        int fd = -1;
        for(auto const* a = list; a != nullptr; a = a->ai_next) {
            fd = ::socket(a->ai_family, a->ai_socktype, a->ai_protocol);
            if(fd < 0) { continue; }
            if(::connect(fd, a->ai_addr, a->ai_addrlen) == 0) { break; }
            ::close(fd);
            fd = -1;
        }
        ::freeaddrinfo(list);
        return fd;
    }

    void run() {
        while(!stop_) {
            auto const fd = open();
            if(fd < 0) {
                std::this_thread::sleep_for(500ms);
                continue;
            }
            {
                std::lock_guard const lock{mutex_};
                fd_ = fd;
            }
            connected_ = true;
            onChange_();
            send(Protocol::Command{Protocol::Get{}});
            send(Protocol::Command{Protocol::GetParameters{}});

            std::vector<std::byte> buffer;
            std::byte              chunk[256];
            while(!stop_) {
                auto const n = ::recv(fd, chunk, sizeof(chunk), 0);
                if(n <= 0) { break; }
                buffer.insert(buffer.end(), chunk, chunk + n);
                while(!buffer.empty()) {
                    Protocol::Report report{};
                    auto const       result = Protocol::Packager::unpack(buffer, report);
                    auto const       used   = result ? result->consumed : result.error().consumed;
                    buffer.erase(buffer.begin(),
                                 buffer.begin() + static_cast<std::ptrdiff_t>(used));
                    if(result) {
                        onReport_(report);
                    } else if(used == 0) {
                        break;
                    }
                }
            }
            connected_ = false;
            closeSocket();
            onChange_();
            std::this_thread::sleep_for(500ms);
        }
    }

    std::string                                  host_;
    std::string                                  port_;
    std::function<void(Protocol::Report const&)> onReport_;
    std::function<void()>                        onChange_;
    std::mutex                                   mutex_;
    int                                          fd_{-1};
    std::atomic<bool>                            connected_{false};
    std::atomic<bool>                            stop_{false};
    std::thread                                  thread_;
};

constexpr std::array<std::string_view, 5> Modes{"idle",
                                                "moving",
                                                "calibrating",
                                                "FAULT",
                                                "by hand (buttons on the board)"};
constexpr std::array<std::string_view, 3> Faults{"none",
                                                 "timeout: the limit did not come",
                                                 "both limits at once: wiring"};
constexpr std::array<std::string_view, 7> RegulatorStates{
  "off (not enabled, or no target)",
  "blocked (calibration, fault, by hand, target on the display)",
  "BLIND: no reading",
  "homing: to the cold limit",
  "watching",
  "moving",
  "settling"};
constexpr std::array<std::string_view, 7>
  Views{"temperature", "position", "temperature, blinking", "CAL", "Err", "target", "HAn"};

template<typename E,
         std::size_t N>
std::string_view name(std::array<std::string_view,
                                 N> const& names,
                      E                    value) {
    auto const i = static_cast<std::size_t>(value);
    return i < N ? names[i] : std::string_view{"?"};
}

/// The MAX31865's fault status register (07h), bit by bit: data sheet table 7, "Fault Status
/// Register Definition" (D7 .. D2; D1, D0 unused).
std::string rtdFaultText(std::uint8_t fault) {
    static constexpr std::array<std::pair<std::uint8_t, std::string_view>, 6> Bits{
      {{0x80, "RTD high (open?)"},
       {0x40, "RTD low (short?)"},
       {0x20, "REFIN- high"},
       {0x10, "REFIN- low"},
       {0x08, "RTDIN- low"},
       {0x04, "over/undervoltage"}}
    };
    std::string text;
    for(auto const& [bit, what] : Bits) {
        if((fault & bit) != 0) {
            if(!text.empty()) { text += ", "; }
            text += what;
        }
    }
    return text.empty() ? "none" : text;
}

// The one place quantities become numbers for a person: on the host a double is free.
template<auto R,
         typename Rep>
double in(mp_units::quantity<R,
                             Rep> q,
          mp_units::Unit auto     unit) {
    return mp_units::value_cast<double>(q).numerical_value_in(unit);
}

double degC(auto q) { return in(q, mp_units::si::degree_Celsius); }

double kelvin(auto q) { return in(q, mp_units::si::kelvin); }

double percentOf(auto q) { return in(q, mp_units::percent); }

Valve::Position positionFromPercent(double percent) {
    return Units::position(static_cast<std::int32_t>(std::lround(percent * 1000.0)));
}

std::string positionText(Protocol::State const& s) {
    if(!s.position.known) { return s.calibrated ? "not known (no limit seen yet)" : "not known"; }
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.1f %% open", percentOf(s.position.value));
    return buffer;
}

std::string temperatureText(Protocol::State const& s) {
    if(!s.temperature.known) { return "no reading"; }
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.2f C", degC(s.temperature.value));
    return buffer;
}

std::string targetText(Valve::Target target) {
    if(target == Valve::Off) { return "off"; }
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.1f C", degC(target));
    return buffer;
}

std::optional<std::size_t> parameterIndex(std::string_view name) {
    for(std::size_t i = 0; i < Regulator::ParameterCount; ++i) {
        if(Regulator::Info[i].name == name) { return i; }
    }
    return std::nullopt;
}

/// `--dump [green|red|calibrate ...] [seconds=N] [socket=PATH | port=N]`: no screen. Sends the presses, asks for the
/// State and prints every one that comes back for N seconds (2) - for scripts.
int dump(int    argc,
         char** argv) {
    std::string host;
    std::string port = TcpPort;
    for(int i = 2; i < argc; ++i) {
        std::string_view const arg{argv[i]};
        if(arg.starts_with("port=")) {
            host = TcpHost;
            port = arg.substr(5);
        } else if(arg.starts_with("socket=")) {
            host = arg.substr(7);
        }
    }
    std::mutex print;
    Link       link{host,
                    port,
                    [&](Protocol::Report const& report) {
                  std::lock_guard const lock{print};
                  if(auto const* r = std::get_if<Protocol::Regulation>(&report)) {
                      auto const& st = r->status;
                      std::printf(
                        "regulation state=%u flowing=%d correcting=%d fast=%d atLimit=%d "
                        "unanswered=%u milliDegC=%d errorMilli=%d fastSlope=%d "
                        "slowSlope=%d runs=%u\n",
                        static_cast<unsigned>(st.state),
                        st.flowing,
                        st.correcting,
                        st.fast,
                        st.atLimit,
                        st.noResponses,
                        Units::raw(st.reading),
                        Units::raw(st.error),
                        Units::raw(st.fastSlope),
                        Units::raw(st.slowSlope),
                        st.runs);
                      std::fflush(stdout);
                      return;
                  }
                  if(auto const* l = std::get_if<Protocol::Learned>(&report)) {
                      std::printf("learned");
                      for(std::size_t i = 0; i < Regulator::Bins; ++i) {
                          std::printf(" gain%zu=%u/%u",
                                      i,
                                      unsigned{Units::raw(l->learned.gain[i])},
                                      l->learned.samples[i]);
                      }
                      std::printf(" backlash=%u/%u\n",
                                  unsigned{Units::raw(l->learned.backlash)},
                                  l->learned.backlashSamples);
                      std::fflush(stdout);
                      return;
                  }
                  if(auto const* p = std::get_if<Protocol::Parameters>(&report)) {
                      std::printf("parameters");
                      for(std::size_t i = 0; i < Regulator::ParameterCount; ++i) {
                          std::printf(" %.*s=%d",
                                      static_cast<int>(Regulator::Info[i].name.size()),
                                      Regulator::Info[i].name.data(),
                                      p->values[i]);
                      }
                      std::printf("\n");
                      std::fflush(stdout);
                      return;
                  }
                  auto const& s = std::get<Protocol::State>(report);
                  std::printf(
                    "state mode=%.*s fault=%u power=%d direction=%s limitOpen=%d "
                    "limitClose=%d calibrated=%d openMs=%u closeMs=%u position=%d "
                    "temperatureValid=%d milliDegC=%d rtdFault=0x%02x rtdFaults=%u "
                    "view=%u blink=%d target=%d shownTarget=%d\n",
                    static_cast<int>(name(Modes, s.mode).size()),
                    name(Modes, s.mode).data(),
                    static_cast<unsigned>(s.fault),
                    s.power,
                    s.direction ? "open" : "close",
                    s.limitOpen,
                    s.limitClose,
                    s.calibrated,
                    s.calibration.open.count(),
                    s.calibration.close.count(),
                    s.position.known ? Units::raw(s.position.value) : -1,
                    s.temperature.known,
                    Units::raw(s.temperature.value),
                    s.rtdFault,
                    s.rtdFaults,
                    static_cast<unsigned>(s.view),
                    s.blink,
                    Units::raw(s.target),
                    Units::raw(s.shownTarget));
                  std::fflush(stdout);
                    },
                    [] {}};
    for(int tries = 0; tries < 20 && !link.connected(); ++tries) {
        std::this_thread::sleep_for(100ms);
    }
    if(!link.connected()) {
        std::fprintf(stderr,
                     "cannot connect to %s - is `just log` running, and no other tui?\n",
                     whereText(host, port).c_str());
        return 1;
    }
    int seconds = 2;
    for(int i = 2; i < argc; ++i) {
        std::string_view const arg{argv[i]};
        if(arg == "green") {
            link.send(Protocol::Command{Protocol::Press{Valve::Button::green}});
        } else if(arg == "red") {
            link.send(Protocol::Command{Protocol::Press{Valve::Button::red}});
        } else if(arg == "calibrate") {
            link.send(Protocol::Command{Protocol::Calibrate{}});
        } else if(arg == "hold-green") {
            link.send(Protocol::Command{Protocol::Hold{Valve::Button::green}});
        } else if(arg == "hold-red") {
            link.send(Protocol::Command{Protocol::Hold{Valve::Button::red}});
        } else if(arg.starts_with("target=")) {
            // degrees, "off" is 0
            auto const value = arg.substr(7);
            link.send(Protocol::Command{Protocol::SetTarget{Units::target(static_cast<std::int16_t>(
              value == "off" ? 0L : std::lround(std::atof(value.data()) * 10)))}});
        } else if(arg.starts_with("port=") || arg.starts_with("socket=")) {
            // taken above
        } else if(arg.starts_with("seconds=")) {
            seconds = std::atoi(arg.substr(8).data());
        } else if(arg == "defaults") {
            link.send(Protocol::Command{Protocol::ResetParameters{}});
        } else if(arg == "stop") {
            link.send(Protocol::Command{Protocol::Stop{}});
        } else if(arg.starts_with("goto=")) {
            link.send(Protocol::Command{
              Protocol::GoTo{positionFromPercent(std::atof(arg.substr(5).data()))}});
        } else if(arg == "forget") {
            link.send(Protocol::Command{Protocol::ResetLearned{}});
        } else if(auto const index = parameterIndex(arg.substr(0, arg.find('=')));
                  index && arg.find('=') != std::string_view::npos)
        {
            link.send(Protocol::Command{
              Protocol::SetParameter{
                                     static_cast<std::uint8_t>(*index),
                                     static_cast<std::int16_t>(std::atoi(arg.substr(arg.find('=') + 1).data()))}
            });
        } else {
            std::fprintf(stderr,
                         "unknown: %s (green, red, hold-green, hold-red, calibrate, "
                         "target=DEG|off, goto=PERCENT, stop, <parameter>=VALUE, defaults, forget, "
                         "seconds=N, socket=PATH, "
                         "port=N)\n",
                         argv[i]);
            return 1;
        }
    }
    std::this_thread::sleep_for(std::chrono::seconds{seconds});
    return 0;
}

}   // namespace

int main(int    argc,
         char** argv) {
    using namespace ftxui;

    if(argc > 1 && std::string_view{argv[1]} == "--dump") { return dump(argc, argv); }

    std::string const host = argc > 1 ? argv[1] : "";
    std::string const port = argc > 2 ? argv[2] : TcpPort;

    auto screen = ScreenInteractive::Fullscreen();

    std::mutex           mutex;
    Protocol::State      state{};
    Protocol::Regulation regulation{};
    Regulator::Values    parameters{};
    Regulator::Learned   learned{};
    bool                 haveState{false};
    bool                 haveParameters{false};
    std::string          lastSent{"-"};
    int                  picked{0};

    Link link{host,
              port,
              [&](Protocol::Report const& report) {
                  {
                      std::lock_guard const lock{mutex};
                      if(auto const* s = std::get_if<Protocol::State>(&report)) {
                          state     = *s;
                          haveState = true;
                      } else if(auto const* r = std::get_if<Protocol::Regulation>(&report)) {
                          regulation = *r;
                      } else if(auto const* p = std::get_if<Protocol::Parameters>(&report)) {
                          parameters     = p->values;
                          haveParameters = true;
                      } else if(auto const* l = std::get_if<Protocol::Learned>(&report)) {
                          learned = l->learned;
                      }
                  }
                  screen.PostEvent(Event::Custom);
              },
              [&] {
                  {
                      std::lock_guard const lock{mutex};
                      haveState      = false;
                      haveParameters = false;
                  }
                  screen.PostEvent(Event::Custom);
              }};

    auto const send = [&](Protocol::Command const& command, std::string what) {
        link.send(command);
        std::lock_guard const lock{mutex};
        lastSent = std::move(what);
    };

    auto green = Button(" [g] green: target down ", [&] {
        send(Protocol::Command{Protocol::Press{Valve::Button::green}}, "green");
    });
    auto red = Button(" [r] red: target up ",
                      [&] { send(Protocol::Command{Protocol::Press{Valve::Button::red}}, "red"); });
    auto holdGreen = Button(" [G] motor: open ", [&] {
        send(Protocol::Command{Protocol::Hold{Valve::Button::green}}, "hold green");
    });
    auto holdRed   = Button(" [R] motor: close ", [&] {
        send(Protocol::Command{Protocol::Hold{Valve::Button::red}}, "hold red");
    });
    // Calibration runs the motor end to end twice: not on a single key. Nor are all
    // parameters thrown away on one.
    bool armedDefaults = false;
    bool armedForget   = false;
    // the position entry (p): what has been typed so far
    bool        entering = false;
    std::string entry;
    bool        armed = false;
    auto calibrate = Button(" [c] [c] calibrate ",
                            [&] { send(Protocol::Command{Protocol::Calibrate{}}, "calibrate"); });
    auto stop    = Button(" [s] STOP ", [&] { send(Protocol::Command{Protocol::Stop{}}, "stop"); });
    auto buttons = Container::Horizontal({green, red, holdGreen, holdRed, stop, calibrate});

    auto view = Renderer(buttons, [&] {
        Protocol::State      s;
        Protocol::Regulation reg;
        Regulator::Values    values;
        Regulator::Learned   learnt;
        bool                 have;
        bool                 haveValues;
        std::string          sent;
        int                  pickedNow;
        {
            std::lock_guard const lock{mutex};
            s          = state;
            reg        = regulation;
            values     = parameters;
            learnt     = learned;
            have       = haveState;
            haveValues = haveParameters;
            sent       = lastSent;
            pickedNow  = picked;
        }
        auto const row = [](std::string label, Element value) {
            return hbox({text(std::move(label)) | size(WIDTH, EQUAL, 14), std::move(value)});
        };
        auto const lamp = [](bool on, std::string label) {
            return text((on ? " ● " : " ○ ") + std::move(label)) | (on ? bold : dim);
        };

        Elements rows;
        if(!link.connected()) {
            rows.push_back(
              text("not connected to " + whereText(host, port) + " - is `just log` running?")
              | color(Color::Red));
        } else if(!have) {
            rows.push_back(text("connected, waiting for the valve's State ...") | dim);
        } else {
            auto mode = text(std::string{name(Modes, s.mode)}) | bold;
            if(s.mode == Valve::Mode::fault) { mode = mode | color(Color::Red); }
            if(s.mode == Valve::Mode::moving) { mode = mode | color(Color::Green); }
            if(s.mode == Valve::Mode::calibrating) { mode = mode | color(Color::Yellow); }
            rows.push_back(row("mode", mode));
            rows.push_back(row("fault", text(std::string{name(Faults, s.fault)})));
            rows.push_back(
              row("relays",
                  hbox({lamp(s.power, "power"),
                        lamp(s.direction, s.direction ? "direction: open" : "direction: close")})));
            rows.push_back(
              row("limits", hbox({lamp(s.limitOpen, "open"), lamp(s.limitClose, "close")})));
            rows.push_back(row("position", text(positionText(s))));
            rows.push_back(gauge(s.position.known
                                   ? static_cast<float>(percentOf(s.position.value) / 100.0)
                                   : 0.F)
                           | color(s.position.known ? Color::Blue : Color::GrayDark));
            rows.push_back(
              row("calibration",
                  text(s.calibrated
                         ? "open " + std::to_string(s.calibration.open.count()) + " ms, close "
                             + std::to_string(s.calibration.close.count()) + " ms"
                         : "none: calibrate first")));
            rows.push_back(separator());
            rows.push_back(row("temperature", text(temperatureText(s))));
            rows.push_back(row(
              "rtd fault",
              text(rtdFaultText(s.rtdFault) + "  (" + std::to_string(s.rtdFaults) + " rejected)")));
            rows.push_back(row("target", text(targetText(s.target))));
            rows.push_back(
              row("display",
                  text(std::string{name(Views, s.view)}
                       + (s.view == Valve::View::Kind::target ? ": " + targetText(s.shownTarget)
                                                              : std::string{})
                       + (s.blink ? " (blinking)" : ""))));

            auto const& st    = reg.status;
            auto const  fixed = [](double value, char const* format) {
                char buffer[48];
                std::snprintf(buffer, sizeof(buffer), format, value);
                return std::string{buffer};
            };
            rows.push_back(separator());
            auto regulatorState = text(std::string{name(RegulatorStates, st.state)}) | bold;
            if(st.state == Regulator::State::blind) {
                regulatorState = regulatorState | color(Color::Red);
            }
            rows.push_back(row("regulator", regulatorState));
            rows.push_back(row("",
                               hbox({lamp(st.flowing, "water flows"),
                                     lamp(st.correcting, "correcting"),
                                     lamp(st.fast, "FAST"),
                                     lamp(st.atLimit, "end of travel")})));
            rows.push_back(row("error",
                               text(s.target == Valve::Off
                                      ? std::string{"-"}
                                      : fixed(kelvin(st.error), "%+.2f K") + "   (filtered reading "
                                          + fixed(degC(st.reading), "%.2f C)"))));
            rows.push_back(row("slope",
                               text(fixed(Units::raw(st.fastSlope) / 1000.0, "%+.3f K/s fast,  ")
                                    + fixed(Units::raw(st.slowSlope) / 1000.0, "%+.4f K/s slow"))));
            rows.push_back(
              row("runs",
                  text(std::to_string(st.runs) + " since boot, " + std::to_string(st.noResponses)
                       + " in a row without an answer")));
        }

        Elements list;
        if(!haveValues) {
            list.push_back(text("no parameters yet") | dim);
        } else {
            for(std::size_t i = 0; i < Regulator::ParameterCount; ++i) {
                auto const& info = Regulator::Info[i];
                auto        line
                  = hbox({text(std::string{info.name}) | size(WIDTH, EQUAL, 14),
                          text(std::to_string(values[i])) | size(WIDTH, EQUAL, 7),
                          text(std::string{info.unit}) | size(WIDTH, EQUAL, 6),
                          text(values[i] == info.initial ? std::string{}
                                                         : "(" + std::to_string(info.initial) + ")")
                            | dim});
                if(static_cast<int>(i) == pickedNow) { line = line | inverted; }
                list.push_back(std::move(line));
            }
            auto const& info = Regulator::Info[static_cast<std::size_t>(pickedNow)];
            list.push_back(separator());
            list.push_back(paragraph(std::string{info.help}));
            list.push_back(text(std::to_string(info.min) + " .. " + std::to_string(info.max)
                                + ", default " + std::to_string(info.initial))
                           | dim);

            // travel per kelvin by region of the travel, 0.01 %/K (clean runs behind it)
            list.push_back(separator());
            list.push_back(text("learned: c%/K by region (runs)") | bold);
            std::string gains;
            for(std::size_t i = 0; i < Regulator::Bins; ++i) {
                gains += learnt.samples[i] == 0 ? std::string{"-"}
                                                : std::to_string(Units::raw(learnt.gain[i])) + "("
                                                    + std::to_string(learnt.samples[i]) + ")";
                gains += i + 1 < Regulator::Bins ? " " : "";
            }
            list.push_back(paragraph(gains));
            list.push_back(text("backlash " + std::to_string(Units::raw(learnt.backlash)) + " d% ("
                                + std::to_string(learnt.backlashSamples) + ")"));
        }

        return vbox({text(" water_mix remote ") | bold | center,
                     separator(),
                     hbox({vbox(std::move(rows)) | flex,
                           separator(),
                           vbox(std::move(list)) | size(WIDTH, EQUAL, 42)}),
                     separator(),
                     buttons->Render(),
                     entering ? hbox({text(" go to position: "),
                                      text(entry + "_") | bold | inverted,
                                      text(" %   Enter runs the motor there, Esc cancels")})
                              : text(" [p] go to a position in percent     [s] / space: stop the "
                                     "motor at once")
                                  | dim,
                     text("parameters: up/down pick, left/right or -/+ by 1, </> by 10, d default, "
                          "D D all defaults, L L forget what was learned")
                       | dim,
                     text(armed ? "press c again to start calibration, any other key cancels"
                          : armedDefaults ? "press D again for ALL defaults, any other key cancels"
                          : armedForget   ? "press L again to forget what was learned"
                                          : "last sent: " + sent + "     q quits")
                       | dim})
             | border;
    });

    auto app = CatchEvent(view, [&](Event event) {
        if(event == Event::Character('q')) {
            screen.Exit();
            return true;
        }
        // the position entry takes every key while it is open
        if(entering) {
            if(event == Event::Escape) {
                entering = false;
            } else if(event == Event::Return) {
                entering = false;
                if(!entry.empty()) {
                    auto const percent = std::clamp(std::atof(entry.c_str()), 0.0, 100.0);
                    send(Protocol::Command{Protocol::GoTo{positionFromPercent(percent)}},
                         "go to " + entry + " %");
                }
            } else if(event == Event::Backspace) {
                if(!entry.empty()) { entry.pop_back(); }
            } else if(event.is_character()) {
                auto const c = event.character();
                if(c.size() == 1 && ((c[0] >= '0' && c[0] <= '9') || c[0] == '.')
                   && entry.size() < 5)
                {
                    entry += c;
                } else if(c == "s" || c == " ") {
                    entering = false;
                    send(Protocol::Command{Protocol::Stop{}}, "stop");
                }
            }
            return true;
        }
        if(event == Event::Character('s') || event == Event::Character(' ')) {
            send(Protocol::Command{Protocol::Stop{}}, "stop");
            return true;
        }
        if(event == Event::Character('p')) {
            entering = true;
            entry.clear();
            return true;
        }

        bool const wasArmed         = armed;
        bool const wasArmedDefaults = armedDefaults;
        bool const wasArmedForget   = armedForget;
        if(event.is_character()) {
            armed         = false;
            armedDefaults = false;
            armedForget   = false;
        }
        if(event == Event::Character('L')) {
            if(wasArmedForget) {
                send(Protocol::Command{Protocol::ResetLearned{}}, "forget what was learned");
            } else {
                armedForget = true;
            }
            return true;
        }

        // the parameters
        auto const change = [&](int by, bool toDefault) {
            std::size_t  index{};
            std::int16_t value{};
            {
                std::lock_guard const lock{mutex};
                if(!haveParameters) { return; }
                index            = static_cast<std::size_t>(picked);
                auto const& info = Regulator::Info[index];
                auto const moved = std::clamp(parameters[index] + by, int{info.min}, int{info.max});
                value            = toDefault ? info.initial : static_cast<std::int16_t>(moved);
            }
            send(
              Protocol::Command{
                Protocol::SetParameter{static_cast<std::uint8_t>(index), value}
            },
              std::string{Regulator::Info[index].name} + " = " + std::to_string(value));
        };
        if(event == Event::ArrowUp || event == Event::ArrowDown) {
            std::lock_guard const lock{mutex};
            picked = std::clamp(picked + (event == Event::ArrowUp ? -1 : 1),
                                0,
                                static_cast<int>(Regulator::ParameterCount) - 1);
            return true;
        }
        if(event == Event::ArrowLeft || event == Event::Character('-')) {
            change(-1, false);
            return true;
        }
        if(event == Event::ArrowRight || event == Event::Character('+')) {
            change(1, false);
            return true;
        }
        if(event == Event::Character('<')) {
            change(-10, false);
            return true;
        }
        if(event == Event::Character('>')) {
            change(10, false);
            return true;
        }
        if(event == Event::Character('d')) {
            change(0, true);
            return true;
        }
        if(event == Event::Character('D')) {
            if(wasArmedDefaults) {
                send(Protocol::Command{Protocol::ResetParameters{}}, "all parameters to defaults");
            } else {
                armedDefaults = true;
            }
            return true;
        }
        if(event == Event::Character('g')) {
            send(Protocol::Command{Protocol::Press{Valve::Button::green}}, "green");
            return true;
        }
        if(event == Event::Character('r')) {
            send(Protocol::Command{Protocol::Press{Valve::Button::red}}, "red");
            return true;
        }
        if(event == Event::Character('G')) {
            send(Protocol::Command{Protocol::Hold{Valve::Button::green}}, "hold green");
            return true;
        }
        if(event == Event::Character('R')) {
            send(Protocol::Command{Protocol::Hold{Valve::Button::red}}, "hold red");
            return true;
        }
        if(event == Event::Character('c')) {
            if(wasArmed) {
                send(Protocol::Command{Protocol::Calibrate{}}, "calibrate");
            } else {
                armed = true;
            }
            return true;
        }
        return false;
    });

    screen.Loop(app);
    return 0;
}
