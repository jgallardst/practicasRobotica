#include "Timer.h"

Timer::Timer()
{

}

Timer::~Timer()
{
	running = false;
	if(running) this->stop();
}


void Timer::setPeriod(int p)
{
	this->period = p;
}

void Timer::stop()
{
	running = false;
	if(timer.joinable()) timer.join();
}

void Timer::start(int p)
{
	running = true;
	period = p;
	timer = std::thread(&Timer::run, this);
	
}

void Timer::connect(std::function<void()> callback)
{
	func = callback;
}



bool Timer::isRunning()
{
	return running;
}

