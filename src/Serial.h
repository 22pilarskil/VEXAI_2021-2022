#ifndef INC_7405XV5_SERIAL_H
#define INC_7405XV5_SERIAL_H

#ifdef __cplusplus

#include "json.hpp"
#include "sync.h"
#include <functional>
#include <queue>

extern "C" {
#endif
void serial_initialize();
void serial_background_processing(void*);
#ifdef __cplusplus
}

namespace lib7405x {
    class Serial {
        friend void ::serial_background_processing(void*);
        friend void ::serial_initialize();

    public:
        typedef enum ser_e {
            STDOUT,
            STDERR,
            DISPLAY,
        } ser_e_t;

        typedef struct message_s {
            std::string header, msg;
            ser_e_t stream;
            uint32_t time;
        } message_s_t;

        constexpr static char DELIMITER = '#';
        constexpr static const char* LINK_ID = "VAIC_7405X_link";

    private:
        Serial();
        static Serial* _instance;
        inline Serial* operator->() {
            return _instance;
        }

        const pros::mutex_t _write_mutex = pros::c::mutex_create();
        const pros::mutex_t _callback_mutex = pros::c::mutex_create();

        std::queue<message_s_t> _outbound{};
        std::unordered_map<std::string, std::function<void(nlohmann::json)>> _callbacks{};

        [[noreturn]] void process();

        [[noreturn]] static void internal_read(void*);
        [[noreturn]] static void vexlink_read(void*);
        static void internal_write(const message_s_t& msg);

    public:
        static Serial* Instance();
        void send(ser_e_t stream, const std::string& header, const nlohmann::json& msg = nlohmann::json());
        void onReceive(const std::string& header, const std::function<void(nlohmann::json)>& callback);
        void deregisterCallback(const std::string& header);
    };
}// namespace lib7405x
#endif
#endif//INC_7405XV5_SERIAL_H
