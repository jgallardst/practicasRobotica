#include "TelegramHandler.h"

TelegramHandler::TelegramHandler(){
}

void TelegramHandler::sendTagFound(){
    Bot bot(token);
    bot.getApi().sendMessage(14877845, "Found AprilTag");
}