/**
 * NRF52 timer emulator class definition
 */

#include <thread>
#include "nrf52-timer-emulator.h"

// Constructor
NRF52_MBED_Timer::NRF52_MBED_Timer(int timer) {
    // Do nothing with the argument
}

// Register timer ISR callback
bool NRF52_MBED_Timer::attachInterruptInterval(const unsigned long interval, 
                                                timer_func_ptr cb) {
    
    // Assign callback if there is not one already
    if (timer_cb_ptr_ != 0) {
        return false;
    } else {
        timer_cb_ptr_ = cb;
    }

    // Remember the interval
    interval_ = interval;

    // Spawn timer thread
    std::thread timer_thread (&NRF52_MBED_Timer::start_timer_thread, this);

    return true;
}

// Timer thread
void NRF52_MBED_Timer::start_timer_thread() {

    // Check if we have no callback set
    if (timer_cb_ptr_ == 0) {
        return;
    }

    // TODO
    // - Is this running???

    // Call user callback at set interval
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        timer_cb_ptr_();
    }
}