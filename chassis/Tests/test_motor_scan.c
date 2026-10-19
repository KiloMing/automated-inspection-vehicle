#include "MotorScan_Core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void) {
    MotorScan_Core s={0}; unsigned i;
    MotorScan_CoreStart(&s,100);
    assert(s.state==SCAN_RUNNING && s.pwm==0);
    MotorScan_CoreUpdate(&s,1099); assert(s.pwm==0);
    MotorScan_CoreUpdate(&s,1100); assert(s.pwm==20);
    for(i=2;i<=50;i++) MotorScan_CoreUpdate(&s,100+i*1000);
    assert(s.pwm==1000 && s.state==SCAN_RUNNING);
    MotorScan_CoreUpdate(&s,51099); assert(s.pwm==1000);
    MotorScan_CoreUpdate(&s,51100); assert(s.pwm==0 && s.state==SCAN_DONE);
    MotorScan_CoreStart(&s,0);
    MotorScan_CoreUpdate(&s,5000); assert(s.pwm==20); /* No skipped dwell stages. */
    MotorScan_CoreUpdate(&s,5001); assert(s.pwm==20);
    MotorScan_CoreStop(&s,SCAN_STOPPED); MotorScan_CoreUpdate(&s,9000);
    assert(s.pwm==0 && s.state==SCAN_STOPPED);
    MotorScan_CoreStart(&s,0xFFFFFF00U); MotorScan_CoreUpdate(&s,744U);
    assert(s.pwm==20);
    assert(MotorScan_Delta(5,65530)==11);
    assert(MotorScan_Delta(65530,5)==-11);
    assert(fabsf(MotorScan_RPM(10,20)-12.0f)<0.001f);
    assert(fabsf(MotorScan_Speed(10,20)-0.04f)<0.00001f);
    assert(fabsf(MotorScan_RPM(-25,20)+30.0f)<0.001f);
    assert(fabsf(MotorScan_RPM(2500,1000)-60.0f)<0.001f);
    assert(fabsf(MotorScan_Speed(2500,1000)-0.20f)<0.00001f);
    assert(MotorScan_RPM(1,0)==0 && MotorScan_Speed(1,0)==0);
    puts("Motor scan tests passed"); return 0;
}
