#include "ejemplo1.h"


using namespace std;
ejemplo1::ejemplo1(): Ui_Counter()
{
    setupUi(this);
    num = 0;
    lcdNumber->display(num);
    show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
	t = new Timer();
	void* fn = &ejemplo1::updateNumber;
	this->connect(fn, (void*) this);
	t->start(1000);
}

void ejemplo1::doButton()
{
	if(t->isRunning()) t->stop();
	else t->start(1000);
}

void ejemplo1::updateNumber()
{
	num++;
	lcdNumber->display(num);
}


ejemplo1::~ejemplo1()
{
	delete t;
}
