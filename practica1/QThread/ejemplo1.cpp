#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
    setupUi(this);
    num = 0;
    lcdNumber->display(num);
    show();
	t = new Timer();
	t->startTimer(1000);
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
	connect(t, SIGNAL(timeout()), this, SLOT(updateNumber()));
}

void ejemplo1::doButton()
{
	if(t->running())t->stop();
	else t->startTimer(1000);
}

void ejemplo1::updateNumber(){
    num++;
    lcdNumber->display(num);
}




