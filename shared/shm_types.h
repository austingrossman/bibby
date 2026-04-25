#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
  #include <atomic>
  #define ATOMIC(T) std::atomic<T>
#else
  #include <stdatomic.h>
  #define ATOMIC(T) _Atomic T
#endif

#define SHM_NAME "/biab_heater"

// Frontend writes duty1/duty2/simulate_zc; heater-controller writes the three status fields.
typedef struct {
  ATOMIC(float)    duty1;          // SSR 1 duty cycle [0.0, 1.0]
  ATOMIC(float)    duty2;          // SSR 2 duty cycle [0.0, 1.0]
  ATOMIC(bool)     simulate_zc;    // when set, timeouts are treated as zero crossings (testing)
  ATOMIC(uint32_t) iteration;      // incremented each ZC; frontend uses this to verify controller is alive
  ATOMIC(bool)     output1;        // current fired state of SSR 1
  ATOMIC(bool)     output2;        // current fired state of SSR 2
  ATOMIC(bool)     watchdog_alarm; // set when no zero crossing detected within timeout
} HeaterShm;
