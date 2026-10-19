#include <assert.h>
#include <stdio.h>
#include "MotorPID.h"

int main(void)
{
    MotorPID a, b;
    int i;
    MotorPID_Reset(&a);
    MotorPID_Reset(&b);
    /* Test build: KP=100, KI=50, KD=1. */
    assert(MotorPID_Update(&a, 1.0f, 0.0f) == 101);
    assert(MotorPID_Update(&a, 1.0f, 0.0f) == 102);
    assert(b.integral == 0.0f);
    assert(MotorPID_Update(&a, 0.0f, 1.0f) == 0);
    assert(a.integral == 0.0f && !a.initialized);
    for (i = 0; i < 100; ++i)
        assert(MotorPID_Update(&a, 20.0f, 0.0f) == 1000);
    assert(a.integral == 0.0f);
    MotorPID_Reset(&a);
    assert(MotorPID_Update(&a, 1.0f, 2.0f) == 0);
    assert(a.integral == 0.0f);
    MotorPID_Reset(&a);
    assert(MotorPID_Update(&a, -1.0f, 0.0f) == -101);
    MotorPID_Reset(&a);
    assert(MotorPID_Update(&a, 1.0f, 0.0f) == 101);
    assert(MotorPID_Update(&a, 2.0f, 0.0f) == 203);
    /* Measurement change contributes -50; no derivative on setpoint. */
    assert(MotorPID_Update(&a, 2.0f, 1.0f) == 54);
    puts("Motor PID tests passed");
    return 0;
}
