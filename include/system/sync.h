#ifndef INC_7405XV5_SYNC_H
#define INC_7405XV5_SYNC_H

#include "api.h"
#include <unordered_map>
#include <cstdint>

class Guard {
private:
    pros::mutex_t _lock = pros::c::mutex_create();
    uint8_t loop = 1;

public:
    static std::unordered_map<uintptr_t, pros::mutex_t> _mutexes;
    static void *_arrLock;

    Guard() = delete;

    Guard(const void* obj, uint32_t timeout) {
        if (obj) {
            pros::c::mutex_take(_arrLock, timeout);
            auto it = _mutexes.find((uintptr_t) obj);
            if (it != _mutexes.end()) {
                pros::c::mutex_delete(_lock);
                _lock = it->second;
            } else {
                _mutexes[(uintptr_t) obj] = _lock;
            }
            pros::c::mutex_give(_arrLock);
            if (pros::c::mutex_take(_lock, timeout)) {
                return;
            }
        }
        loop--;
    }

    explicit Guard(const void* obj) : Guard(obj, TIMEOUT_MAX) {}

    uint32_t enter() {
        return loop > 0 ? loop-- : 0;
    }

    ~Guard() {
        pros::c::mutex_give(_lock);
    }
};

#define synchronized(n) for (Guard guard(n); guard.enter();)
#define synchronized_timeout(n, m) for (Guard guard(n, m); guard.enter();)

#endif//INC_7405XV5_SYNC_H
