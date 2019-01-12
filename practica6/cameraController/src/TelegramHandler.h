/*
 *    Copyright (C)2018 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
       \brief
       @author authorname
*/



#ifndef TELEGRAMHANDLER_H
#define TELEGRAMHANDLER_H

#include <tgbot/tgbot.h>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <chrono>

using namespace TgBot;

class TelegramHandler
{
public:
    TelegramHandler();
    void sendTagFound(int tid, int tHamming, float x, float z);
private:
    const std::string token = "672516351:AAGyUF0mwSdJTZHuuywvlUJ1DCXC4-Szx2I";
    const int id = 14877845;
    std::chrono::system_clock::time_point timer = std::chrono::system_clock::now();
};

#endif
