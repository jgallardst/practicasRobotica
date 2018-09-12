#include "Timer.h"

Timer::Timer()
{

}

void Timer::setPeriod(int t)
{
	this->period = t;
}
void Timer::startTimer(int t)
{
	onRun = true;
	this->period = t;
	this->start();
}


void Timer::stop()
{
	onRun = false;
}

bool Timer::running()
{
	return (this->onRun);
}




