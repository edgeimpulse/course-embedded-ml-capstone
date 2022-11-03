/**
 * NRF52 timer emulator class definition
 */

#include "time-emulator.h"
#include "nrf52-timer-emulator.h"

// Constructor (do nothing with the argument)
NRF52_MBED_Timer::NRF52_MBED_Timer(int timer) {
}

// Destructor - make sure thread joins
NRF52_MBED_Timer::~NRF52_MBED_Timer() {
    stop();
}

// Register timer ISR callback and start thread
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

    // start thread
    start();

    return true;
}

// Start thread
void NRF52_MBED_Timer::start() {

    // Make sure there's no existing thread
    if (timer_thread_ == nullptr) {

        // Create a new thread
        running_ = true;
        timer_thread_ = new std::thread(&NRF52_MBED_Timer::run, this);
    }
}

// Stop thread
void NRF52_MBED_Timer::stop() {
    running_ = false;
    if (timer_thread_ != nullptr) {
        if (timer_thread_->joinable()) {
            timer_thread_->join();
        }
        delete timer_thread_;
        timer_thread_ = nullptr;
    }
}

// Timer thread
void NRF52_MBED_Timer::run() {

    unsigned long time_start, time_target, time_actual, to_sleep;

    // Check if we have no callback set
    if (timer_cb_ptr_ == 0) {
        return;
    }

    // Initialize times
    time_start = micros();
    time_target = 0;

    // Call user callback at set interval
    running_ = true;
    while (running_) {

        // Determine how long to sleep for to meet target
        time_target += interval_;
        time_actual = micros() - time_start;
        if (time_actual < time_target) {
            to_sleep = time_target - time_actual;
        } else {
            to_sleep = 0;
        }

        // Sleep and call callback function
        std::this_thread::sleep_for(std::chrono::microseconds(to_sleep));
        timer_cb_ptr_();
    }
}