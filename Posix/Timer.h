#ifndef TIMER_H
#define TIMER_H
#include <thread>

class Timer
{	
	
private:
	int period;
	bool running;
public:
    Timer();
	~Timer();
	bool isRunning();
	int getPeriod();
	std::thread* timer;
	void start(int p, std::thread* t);
	void stop();
	void setPeriod(int p);

};
#endif // ejemplo1_H
