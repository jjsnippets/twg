#ifndef __REALTIME__
#define __REALTIME__

int StartRT(int priority, double dt);
void RT_SleepUntil(double dt);

#endif
