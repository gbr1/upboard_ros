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

float frequency;
std::string frame_id;

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
std::vector<pGpio*> listgpio_input;



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

uint8_t getGpio(uint8_t pin){
    uint8_t value=0;
    int k=0;
    bool found=false;
    while((!found) && (k<listgpio_input.size())){
        if (listgpio_input[k]->getPin()==pin){
            found=true;
            try{
                value = listgpio_input[k]->read();
            }
            catch(std::exception& e){
                ROS_ERROR("%s",e.what());
                value= 254;
            } 
        }
        k++;
    }
    if (!found){
        ROS_WARN("GPIO %d is not defined. Check your configs",pin);
        value=255;
    }
    return value;
}

bool checkDOParam(uint8_t pin){
    int k=0;
    while(k<listgpio_output.size()){
        if (listgpio_output[k]->getPin()==pin){
            return true;
        }
        k++;
    }
    return false;
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
    
    //rate parameter
    if (nh.getParam("upboard_ros/frequency", frequency)){
        ROS_INFO("Node rate is %f Hz",frequency);
    }else{
        frequency=100.0;
        ROS_INFO("[Default] Node rate is %f Hz",frequency);
    }

    //frame id parameter
    if (nh.getParam("upboard_ros/frame_id", frame_id)){
        ROS_INFO("Frame Id is %s",frame_id.c_str());
    }else{
        frame_id="base_link";
        ROS_INFO("[Default] Frame Id is %s",frame_id.c_str());
    }


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

    //load digital input pins
    if (nh.getParam("upboard_ros/DigitalIn",paramlist)){
        for (int i=0; i<paramlist.size(); i++){
            try{
                if (!checkDOParam(paramlist[i])){
                    pGpio* tmp = new pGpio(paramlist[i]);
                    tmp->dir(mraa::DIR_IN);
                    listgpio_input.push_back(tmp);
                    
                    ROS_INFO("Pin %d is configured as Digital Input",paramlist[i]);
                }
                else{
                    ROS_WARN("Pin %d is already configured as Digital Output", paramlist[i]);
                }
            }
            catch(std::exception& e){
                ROS_ERROR("%s",e.what());
            }
        }
    }
    

    ros::Subscriber gpiosub = nh.subscribe("/upboard/gpio/write", 10, gpioCallback);
    ros::Publisher gpiopub = nh.advertise<upboard_ros::ListGpio>("/upboard/gpio/read",10);
    ros::Rate rate(frequency);

    upboard_ros::ListGpio gpio_msg;
    
    while (ros::ok()){
        gpio_msg.gpio.clear();
        for (int i=0; i<listgpio_input.size(); i++){
            upboard_ros::Gpio tmp_msg;
            tmp_msg.pin=listgpio_input[i]->getPin();
            tmp_msg.value=getGpio(tmp_msg.pin);
            gpio_msg.gpio.push_back(tmp_msg);
        }
        gpio_msg.header.stamp=ros::Time::now();
        gpio_msg.header.frame_id=frame_id;
        gpiopub.publish(gpio_msg);
        ros::spinOnce();
        rate.sleep();
    }

    
    //set all outputs to 0 and deallocate gpio
    for (int i=0; i<listgpio_output.size(); i++){
        try{
            listgpio_output[i]->write(0);
            delete listgpio_output[i];
        }
        catch(std::exception& e){
            ROS_ERROR("%s",e.what());
        }
    }

    for (int i=0; i<listgpio_input.size(); i++){
        try{
            delete listgpio_input[i];
        }
        catch(std::exception& e){
            ROS_ERROR("%s", e.what());
        }
    }
    
    return 0;
}