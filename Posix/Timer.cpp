#include "Timer.h"

Timer::Timer()
{

}

Timer::~Timer()
{
	running = false;
	if(isRunning()) this->stop();
}


void Timer::setPeriod(int p)
{
	this->period = p;
}

int Timer::getPeriod()
{
	return period;
}


void Timer::stop()
{
	running = false;
	if(timer->joinable()) timer->join();
	delete timer;
}

void Timer::start(int p, std::thread* t)
{
	running = true;
	period = p;
	timer = t;
	
}

bool Timer::isRunning()
{
	return running;
}

