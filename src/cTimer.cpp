#include "cTimer.h"
#include <stdio.h> 
#include <time.h>

namespace MultiColSLAM
{
	unsigned long cTimer::getCostTimeStart()
	{
		struct timespec ts;
    	clock_gettime(CLOCK_MONOTONIC, &ts);
    	return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
	}

	void cTimer::getCostTimeEnd(unsigned long startTime)
	{
		struct timespec ts;
    	clock_gettime(CLOCK_MONOTONIC, &ts);
        unsigned long endTime = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
        printf("cost Time:%lu\r\n", endTime - startTime);
	}

}