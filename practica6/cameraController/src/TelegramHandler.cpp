#include "TelegramHandler.h"

TelegramHandler::TelegramHandler(){
}

void TelegramHandler::sendTagFound(int tid, int tHamming, int x, int z){
    Bot bot(token);

    // Print date
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::stringstream date, msg;
    date << std::put_time(&tm, "[%d-%m-%Y %H:%M:%S]");
    bot.getApi().sendMessage(id, date.str());
    msg << "Found AprilTag with id " << tid << "\n" << "Hamming distance: " << tHamming << "\n"
                << "Robot location (X:" <<  x << ",Z:" << z << ")" << "\n";
    bot.getApi().sendMessage(id, msg.str());
}