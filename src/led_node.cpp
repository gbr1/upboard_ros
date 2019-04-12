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
#include <upboard_ros/Led.h>


void setLed(std::string led_name, int status){
    std::string led_cmd="echo ";
    std::stringstream ss;
    ss<<status;
    led_cmd+=ss.str();
    led_cmd+=" > /sys/class/leds/upboard\:";
    led_cmd+=led_name;
    led_cmd+="\:/brightness";
    system(led_cmd.c_str());
}

void setLeds(int status){
    setLed("blue",status);
    setLed("yellow",status);
    setLed("green",status);
    setLed("red",status);
}

void ledsCallback(const upboard_ros::Led & msg){
    uint8_t led = msg.led;
    switch(led){
        case 0:
            setLeds(msg.value);
            break;
        case 1:
            setLed("blue",msg.value);
            break;
        case 2:
            setLed("yellow",msg.value);
            break;
        case 3:
            setLed("green",msg.value);
            break;
        case 4:
            setLed("red",msg.value);
            break;
        default:
            break;
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