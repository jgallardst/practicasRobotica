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



#ifndef TAG_H
#define TAG_H

#include <thread>
#include <utility>

class Tag {
   public:

    void set(RoboCompAprilTags::tag t) {
        std::lock_guard<std::mutex> l(m);
        newTag = true;
        aprilTag = t;
    }
    int id() {
        std::lock_guard<std::mutex> l(m);
        if(newTag){
            newTag = false;
     	    return aprilTag.id;
        } else return -1;
    }
  private:
    mutable std::mutex m;
    RoboCompAprilTags::tag aprilTag;
    bool newTag = false;
};

#endif