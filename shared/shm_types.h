#pragma once
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

#define SHM_NAME "/biab_heater"

// All fields are individually atomic (lock-free on any 32-bit-capable arch).
// Frontend writes duty1/duty2/simulate_zc; heater-controller writes the three status fields.
typedef struct {
    _Atomic float duty1;          // SSR 1 duty cycle [0.0, 1.0]
    _Atomic float duty2;          // SSR 2 duty cycle [0.0, 1.0]
    _Atomic bool     simulate_zc;  // when set, timeouts are treated as zero crossings (testing)
    _Atomic uint32_t iteration;   // incremented each ZC; frontend uses this to verify controller is alive
    _Atomic bool     output1;     // current fired state of SSR 1
    _Atomic bool     output2;     // current fired state of SSR 2
    _Atomic bool     watchdog_alarm; // set when no zero crossing detected within timeout
} HeaterShm;
