#ifndef TIMER_H
#define TIMER_H
#include <functional>
#include <thread>

class Timer
{	
	
private:
	int period;
	bool running;
public:
    Timer();
	~Timer();
	std::thread timer;
	std::function<void()> func;
	bool isRunning();
	void connect(std::function<void()> callback);
	void start(int p);
	void stop();
	void setPeriod(int p);
	void run(){
        while(running){
            std::this_thread::sleep_for( std::chrono::milliseconds(period) );
			if(running) func();
        }
    }
    
};
#endif // ejemplo1_H
