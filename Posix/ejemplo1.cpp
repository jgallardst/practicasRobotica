#include "ejemplo1.h"


using namespace std;
ejemplo1::ejemplo1(): Ui_Counter()
{
    setupUi(this);
    num = 0;
    lcdNumber->display(num);
    show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
	start(1000);
}

void ejemplo1::doButton()
{
	if(running) stop();
	else start(1000);
}

void ejemplo1::updateNumber()
{
	num++;
	lcdNumber->display(num);
}

void ejemplo1::setPeriod(int p)
{
	this->period = p;
}

void ejemplo1::stop()
{
	running = false;
	if(timer.joinable()) timer.join();
}

void ejemplo1::start(int p)
{
	running = true;
	period = p;
	timer = std::thread(&ejemplo1::run,this);
}

ejemplo1::~ejemplo1()
{
	if(running) this->stop();
}


