#ifndef ejemplo1_H
#define ejemplo1_H
#include <thread>
#include <unistd.h>
#include "ui_counterDlg.h"

class ejemplo1 : public QWidget, public Ui_Counter
{
Q_OBJECT
	
public:
    ejemplo1();
	~ejemplo1();
	bool running;
	int num;
	int period;
	void updateNumber();
	void setPeriod(int p);
 	void start(int p);
	void stop();
	std::thread timer;
	void run(){
        while(running){
            std::this_thread::sleep_for( std::chrono::milliseconds(period) );
			if(running)updateNumber();
        }
    }

    
public slots:
	void doButton();
};

#endif // ejemplo1_H
