/*
 * The MIT License
 *
 * Copyright (c) 2019 Giovanni di Dio Bruno https://gbr1.github.io
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <ros/ros.h>
#include <upboard_ros/Leds.h>

#include "mraa/common.hpp"
#include "mraa/led.hpp"


mraa::Led blue("blue");
mraa::Led yellow("yellow");
mraa::Led green("green");
mraa::Led red("red");

void setLeds(int status){
    blue.setBrightness(status);
    yellow.setBrightness(status);
    green.setBrightness(status);
    red.setBrightness(status);
}

void ledsCallback(const upboard_ros::Leds & msg){
    for (int i=0; i<msg.leds.size(); i++){
        uint8_t led = msg.leds[i].led;
        uint8_t value = msg.leds[i].value;
        switch(led){
            case 0:
                setLeds(value);
                break;
            case 1:
                blue.setBrightness(value);
                break;
            case 2:
                yellow.setBrightness(value);
                break;
            case 3:
                green.setBrightness(value);
                break;
            case 4:
                red.setBrightness(value);
                break;
            default:
                break;
        }
    }
}

int main(int argc, char **argv){
    setLeds(0);
    ros::init(argc, argv, "leds_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/upboard/leds", 10, ledsCallback);
    ros::spin();
    setLeds(0);
    return 0;
}