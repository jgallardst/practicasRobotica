#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
    setupUi(this);
    num = 0;
    lcdNumber->display(num);
    show();
    timer = new QTimer();
    timer->start(1000);
    connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
    connect(timer, SIGNAL(timeout()), this, SLOT(updateNumber()));
}

ejemplo1::~ejemplo1()
{}

void ejemplo1::doButton()
{
    if(timer->isActive())timer->stop();
    else timer->start(1000);
}

void ejemplo1::updateNumber(){
    num++;
    lcdNumber->display(num);
}


