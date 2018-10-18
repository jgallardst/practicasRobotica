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



#ifndef TARGET_H
#define TARGET_H

#include <thread>
#include <utility>

struct target_t {
  int x;
  int z;
};

class Target {
   public:

    void set(int x, int z) {
        std::lock_guard<std::mutex> l(m);
        activo = true;
        pos.x = x; pos.z = z;
    }
    void setInactive(){
        std::lock_guard<std::mutex> l(m);
        activo = false;
    }
    std::tuple<bool, target_t> activoAndGet() {
        std::lock_guard<std::mutex> l(m);
     	if(activo){
            return std::make_tuple(true, pos);
        } 
        else return std::make_tuple(false, pos);
    }
  private:
    mutable std::mutex m;
    target_t pos; 
    bool activo = false;
 
};

#endif