#include "Serial.h"
#include "display.h"

extern "C" int32_t inp_buffer_available();


lib7405x::Serial* lib7405x::Serial::_instance = nullptr;

lib7405x::Serial* lib7405x::Serial::Instance() {
    if (!_instance) {
        _instance = new Serial;
    }
    return _instance;
}

void serial_initialize() {
    pros::Task serial_task_handle(serial_background_processing, nullptr, TASK_PRIORITY_DEFAULT + 1, TASK_STACK_DEPTH_DEFAULT, "lib Serial Daemon");
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
void serial_background_processing(void* ign) {
    lib7405x::Serial::Instance()->process();
}
#pragma GCC diagnostic pop

lib7405x::Serial::Serial() {
    pros::Task serial_read_task_handle(Serial::internal_read, nullptr, TASK_PRIORITY_DEFAULT + 1, TASK_STACK_DEPTH_DEFAULT, "STDIN Read Daemon");
    Guard::_mutexes[(uintptr_t) _write_mutex] = _write_mutex;
    Guard::_mutexes[(uintptr_t) _callback_mutex] = _callback_mutex;
}

[[noreturn]] void lib7405x::Serial::process() {
    uint32_t time = pros::c::millis();
    while (true) {
        if (!pros::c::mutex_take(_write_mutex, 2)) {
            continue;
        }
        nlohmann::json sout_pkt;
        nlohmann::json serr_pkt;
        while (!_outbound.empty()) {
            message_s_t msg = _outbound.front();
            switch (msg.stream) {
                case STDOUT:
                    sout_pkt[msg.header] = msg.msg;
                    break;
                case STDERR:
                    serr_pkt[msg.header] = msg.msg;
                    break;
                case DISPLAY:
                    internal_write(msg);
                    break;
            }
            _outbound.pop();
        }
        if (!sout_pkt.is_null()) {
            internal_write({"v5 SOUT", sout_pkt.dump(), STDOUT, pros::c::millis()});
        }
        if (!serr_pkt.is_null()) {
            internal_write({"v5 SERR", serr_pkt.dump(), STDERR, pros::c::millis()});
        }

        pros::c::mutex_give(_write_mutex);
        pros::c::task_delay_until(&time, 5);
    }
}


[[noreturn]] void lib7405x::Serial::internal_read(void* ign) {
    uint32_t MAX_SIZE = 255, CURRENT_SIZE = 0, retval;
    char* buf = static_cast<char*>(malloc(MAX_SIZE));
    uint32_t time = pros::c::millis();
    while (true) {
        retval = inp_buffer_available();
        if (retval > 0) {
            int c;
            while ((c = getc(stdin)) != EOF && c != '\n') {
                if (CURRENT_SIZE == MAX_SIZE - 1) {
                    buf = static_cast<char*>(realloc(buf, CURRENT_SIZE + MAX_SIZE));
                    MAX_SIZE += CURRENT_SIZE;
                }
                buf[CURRENT_SIZE++] = c;
            }
            if (c == '\n') {
                buf[CURRENT_SIZE] = '\0';
                std::string str(strdup(buf));
                CURRENT_SIZE = 0;
                if (!str.empty()) {
                    auto header = str.substr(0, str.find(DELIMITER));
                    auto msg = str.substr(str.find(DELIMITER) + 1, str.size());
                    // display::Terminal::serial->println(std::string("[Time ").append(std::to_string(pros::c::millis() / 1000)).append(", STDIN]: ").append(header).append(std::string(" ")).append(msg));
                    if (pros::c::mutex_take(Serial::Instance()->_callback_mutex, 2)) {
                        if (Serial::Instance()->_callbacks.find(header) != Serial::Instance()->_callbacks.end()) {
                            auto tempstr = nlohmann::json(msg).get<std::string>();
                            if (tempstr[tempstr.length()] == 'r') {// if there is a return character (depending on your serial settings I guess)
                                Serial::Instance()->_callbacks.at(header)(nlohmann::json::parse(tempstr.substr(0, tempstr.length() - 1)));
                            }
                            Serial::Instance()->_callbacks.at(header)(nlohmann::json(nlohmann::json::parse(tempstr)));
                        }
                        pros::c::mutex_give(Serial::Instance()->_callback_mutex);
                    }
                }
            }
        }
        pros::c::task_delay_until(&time, 5);
    }
}
#pragma GCC diagnostic pop

void lib7405x::Serial::internal_write(const lib7405x::Serial::message_s_t& msg) {
    std::string str = msg.header + DELIMITER + msg.msg;
    std::string wireless_str = "\002" + str + "\n";

    switch (msg.stream) {
        case STDOUT:
            std::cout << str << std::endl;
            break;

        case STDERR:
            std::cerr << str << std::endl;
            break;

        // case DISPLAY:
        //     display::Terminal::logs->println(std::string("[Time ").append(std::to_string(pros::c::millis() / 1000)).append("]: ").append(msg.header));

        default:
            break;
    }
}

void lib7405x::Serial::send(ser_e_t stream, const std::string& header, const nlohmann::json& msg) {
    synchronized(_write_mutex) {
        _outbound.push(message_s_t{header, msg.dump(-1), stream, pros::c::millis()});
    }
}

void lib7405x::Serial::onReceive(const std::string& header, const std::function<void(nlohmann::json)>& callback) {
    synchronized(_callback_mutex) {
        _callbacks[header] = callback;
    }
}

void lib7405x::Serial::deregisterCallback(const std::string& header) {
    synchronized(_callback_mutex) {
        if (_callbacks.find(header) != _callbacks.end()) {
            _callbacks.erase(header);
        }
    }
}

