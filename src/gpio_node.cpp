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

#include "mraa/common.hpp"
#include "mraa/gpio.hpp"

#include <upboard_ros/Gpio.h>
#include <upboard_ros/ListGpio.h>

#include <vector>

class pGpio : public mraa::Gpio{
    private:
        int _pin;
        
    public:
            pGpio(int pin, bool owner=true, bool raw=false):mraa::Gpio(pin,owner,raw){
                _pin=pin;
            }
        int getPin(){return _pin;}
            ~pGpio() {};   
};

std::vector<pGpio*> listgpio_output;


void setGpio(uint8_t pin, uint8_t value){
    int k=0;
    bool found=false;
    while((!found) && (k<listgpio_output.size())){
        if (listgpio_output[k]->getPin()==pin){
            found=true;
            try{
                listgpio_output[k]->write(value);
            }
            catch(std::exception& e){
                ROS_ERROR("%s",e.what());
            } 
        }
        k++;
    }
    if (!found){
        ROS_WARN("GPIO %d is not defined. Check your configs",pin);
    }
}


void gpioCallback(const upboard_ros::ListGpio & msg){
    for (int i=0; i<msg.gpio.size(); i++){
        setGpio(msg.gpio[i].pin,msg.gpio[i].value);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "upboard_ros/gpio_node");
    ros::NodeHandle nh;
    
    /* rostopic pub /upboard/gpio/write upboard_ros/ListGpio "{header: auto, gpio:[{pin: 22, value: 0}]}" --once */
    
    std::vector<int> paramlist;

    //load digital output pins
    if (nh.getParam("upboard_ros/DigitalOut",paramlist)){
        for (int i=0; i<paramlist.size(); i++){
            try{
                pGpio* tmp = new pGpio(paramlist[i]);
                tmp->dir(mraa::DIR_OUT);
                listgpio_output.push_back(tmp);
                
                ROS_INFO("Pin %d is configured as Digital Output",paramlist[i]);
            }
            catch(std::exception& e){
                ROS_ERROR("%s",e.what());
            }
        }
    }
    

    ros::Subscriber gpiosub = nh.subscribe("/upboard/gpio/write", 10, gpioCallback);

    ros::spin();

    
    //put all outputs to 0
    for (int i=0; i<listgpio_output.size(); i++){
        try{
            listgpio_output[i]->write(0);
        }
        catch(std::exception& e){
            ROS_ERROR("%s",e.what());
        }
    }
    
    return 0;
}