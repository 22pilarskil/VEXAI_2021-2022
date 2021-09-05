#include "sync.h"

std::unordered_map<uintptr_t, pros::mutex_t> Guard::_mutexes{};
void *Guard::_arrLock;

__attribute__((constructor(103))) void arr_lock_init() {
    Guard::_arrLock = pros::c::mutex_create();
}