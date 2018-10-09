#ifndef Timer_H
#define Timer_H
#include <QtGui>

class Timer : public QThread
{
Q_OBJECT
public:
    Timer();
	void run() {
		while(onRun){
			msleep(period);
			if(onRun)
				emit timeout();
		}
    }
	void startTimer(int t);
	void stop();
	void setPeriod(int t);
	bool running();
    
private:
	int period;
	bool onRun;
	
signals:
	void timeout();
};


#endif