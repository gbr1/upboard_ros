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