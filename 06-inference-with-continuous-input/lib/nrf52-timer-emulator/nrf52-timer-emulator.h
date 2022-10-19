/**
 * Emulate the NRF52_MBED_Timer library for Arduino.
 * 
 * Author: Shawn Hymel (Edge Impulse)
 * Date: September 29, 2022
 * License: Apache-2.0
 * 
 * Copyright 2022 EdgeImpulse, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef NRF52_TIMER_EMULATOR_H
#define NRF52_TIMER_EMULATOR_H

#define NRF_TIMER_3     3
#define NRF_TIMER_4     4

// ISR callback function pointer types
typedef void (*timer_func_ptr)();

class NRF52_MBED_Timer {
    public:
        NRF52_MBED_Timer(int timer);

        // Arduino interface
        bool attachInterruptInterval(const unsigned long interval, 
                                        timer_func_ptr cb);
    private:
        timer_func_ptr timer_cb_ptr_ = 0;
        unsigned long interval_ = 0;
        void start_timer_thread();

};

#endif // NRF52_IMER_EMULATOR_H