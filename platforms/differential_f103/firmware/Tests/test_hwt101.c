#include "HWT101_Parser.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static void near(float actual, float expected) { assert(fabsf(actual-expected)<0.02f); }
static void feed(HWT101_Parser *p, const uint8_t *bytes, unsigned n, uint32_t ms) {
    unsigned i; for(i=0;i<n;i++) HWT101_ParserFeed(p,bytes[i],ms);
}
static void angle(HWT101_Parser *p, int16_t raw, uint32_t ms) {
    uint8_t f[11]={0x55,0x53,0,0,0,0,0,0,0,0,0}; unsigned i;
    f[6]=(uint8_t)raw; f[7]=(uint8_t)((uint16_t)raw>>8);
    for(i=0;i<10;i++) f[10]=(uint8_t)(f[10]+f[i]);
    feed(p,f,11,ms);
}
int main(void) {
    HWT101_Parser p;
    unsigned i;
    uint8_t bad[11];
    const uint8_t gyro[]={0x55,0x52,0,0,3,0,1,0,0,0,0xAB};
    /* Literal screenshot frame: yaw=0x0D36, version=0x27E5, checksum=0xF7. */
    const uint8_t captured[]={0x55,0x53,0,0,0,0,0x36,0x0D,0xE5,0x27,0xF7};
    const uint8_t junk[]={0x12,0x55,0x53,0x11};
    HWT101_ParserInit(&p);
    assert(!HWT101_ParserZero(&p,0));
    feed(&p,captured,11,100);
    assert(p.data.valid && p.data.sequence==1);
    near(p.data.yaw_deg,18.57788f); near(p.data.heading_deg,0);
    for(i=0;i<11;i++) bad[i]=captured[i];
    bad[10] ^= 1U;
    feed(&p,bad,11,105);
    assert(p.data.sequence==1 && p.data.timestamp_ms==100);
    feed(&p,gyro,11,106);
    assert(p.data.sequence==1);
    /* Shifted/bad frame must not hide the following valid frame. */
    feed(&p,junk,4,110); feed(&p,captured,11,110);
    assert(p.data.sequence==2);
    HWT101_ParserInit(&p);
    angle(&p,32586,100); angle(&p,-32586,200);
    near(p.data.heading_deg,1.99951f);
    near(p.data.continuous_deg,1.99951f);
    near(p.data.yaw_rate_dps,19.9951f);
    assert(p.data.rate_valid);
    near(HWT101_AngleError(-179,179),2);
    near(HWT101_AngleError(179,-179),-2);
    near(HWT101_AngleError(180,0),-180);
    assert(HWT101_ParserZero(&p,200));
    near(p.data.heading_deg,0); near(p.data.continuous_deg,0);
    HWT101_ParserExpire(&p,700);
    assert(!p.data.valid && !p.data.continuous_valid && !p.data.rate_valid);
    angle(&p,-30000,800);
    assert(p.data.valid && !p.data.continuous_valid && !p.data.rate_valid);
    assert(HWT101_ParserZero(&p,800));
    assert(p.data.continuous_valid);
    HWT101_ParserBreak(&p);
    assert(!p.data.valid && !p.data.continuous_valid);
    /* Timer rollover must preserve a short legitimate interval. */
    HWT101_ParserInit(&p); angle(&p,0,0xFFFFFFF0U); angle(&p,16384,84U);
    near(p.data.yaw_rate_dps,900); near(p.data.heading_deg,90);
    /* A broken partial frame times out and a full next frame still parses. */
    HWT101_ParserInit(&p); feed(&p,captured,5,0); feed(&p,captured,11,30);
    assert(p.data.sequence==1);
    /* +90 steps must accumulate past a complete revolution. */
    HWT101_ParserInit(&p); angle(&p,0,0); angle(&p,16384,100);
    angle(&p,-32768,200); angle(&p,-16384,300); angle(&p,0,400);
    near(p.data.continuous_deg,360); near(p.data.heading_deg,0);
    /* Polling after every received byte must allow recovery after timeout. */
    HWT101_ParserExpire(&p,1000);
    for(i=0;i<11;i++) {
        HWT101_ParserFeed(&p,captured[i],1100U+i);
        HWT101_ParserExpire(&p,1100U+i);
    }
    assert(p.data.valid && !p.data.continuous_valid);
    puts("HWT101 parser tests passed"); return 0;
}
