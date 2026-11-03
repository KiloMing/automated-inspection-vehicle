#include "Mecanum.h"
#include <math.h>
void Mecanum_Inverse(const ChassisVelocity *b,WheelVelocity *w){
 float t=CHASSIS_K*b->z;
 w->v[0]=b->x-b->y-t;w->v[1]=b->x+b->y-t;
 w->v[2]=b->x+b->y+t;w->v[3]=b->x-b->y+t;
}
void Mecanum_Forward(const WheelVelocity *w,ChassisVelocity *b){
 b->x=(w->v[0]+w->v[1]+w->v[2]+w->v[3])*0.25f;
 b->y=(-w->v[0]+w->v[1]+w->v[2]-w->v[3])*0.25f;
 b->z=(-w->v[0]-w->v[1]+w->v[2]+w->v[3])/(4*CHASSIS_K);
}
float Mecanum_Limit(WheelVelocity *w,float max){
 unsigned i;float peak=max,scale;
 for(i=0;i<4;i++)if(fabsf(w->v[i])>peak)peak=fabsf(w->v[i]);
 scale=peak>0?max/peak:1;
 for(i=0;i<4;i++)w->v[i]*=scale;return scale;
}
float Mecanum_CountsToMps(float cps){return cps*(6.283185307f*CHASSIS_RADIUS)/CHASSIS_COUNTS_REV;}
float Mecanum_MpsToCounts(float mps){return mps*CHASSIS_COUNTS_REV/(6.283185307f*CHASSIS_RADIUS);}
