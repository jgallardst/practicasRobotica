#ifndef ejemplo1_H
#define ejemplo1_H
#include "Timer.h"

#include "ui_counterDlg.h"

class ejemplo1 : public QWidget, public Ui_Counter
{
Q_OBJECT
public:
    ejemplo1();
	int num;
	Timer* t;
    
public slots:
	void updateNumber();
	void doButton();
};

#endif // ejemplo1_H
