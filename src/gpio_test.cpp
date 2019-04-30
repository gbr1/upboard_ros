#include <ros/ros.h>

#include <upboard_ros/Gpio.h>
#include <upboard_ros/ListGpio.h>

#define BUTTON 24
#define LED    22

#define SLOW   1.0
#define FAST   0.2

upboard_ros::Gpio tmp_msg;
upboard_ros::ListGpio list_msg;

bool status=false;
ros::Time tp;
float blinkrate=SLOW;

ros::Publisher gpiopub;


void gpioCallback(const upboard_ros::ListGpio & msg){
    int k=0;
    bool found=false;
    while((!found) && (k<msg.gpio.size())){
        if (msg.gpio[k].pin==BUTTON){
            found=true;
            if (msg.gpio[k].value!=0){
                blinkrate=FAST;
            } 
            else{
                blinkrate=SLOW;
            }
        }
        k++;
    }
}

void blink(){
    ros::Duration d=ros::Time::now()-tp;
    if (d.toSec()>=blinkrate){
        //create a message to turn on led on pin 22
        tmp_msg.pin=22;
        status=!status;
        tmp_msg.value=status;
        //add to list
        list_msg.gpio.push_back(tmp_msg);
        list_msg.header.stamp=ros::Time::now();
        gpiopub.publish(list_msg);
        list_msg.gpio.clear();
        tp=ros::Time::now();
    }

}

int main(int argc, char **argv){
    ros::init(argc, argv, "gpio_test");
    ros::NodeHandle nh;
    gpiopub = nh.advertise<upboard_ros::ListGpio>("/upboard/gpio/write",10);
    ros::Subscriber gpiosub = nh.subscribe("/upboard/gpio/read", 10, gpioCallback);
    tp=ros::Time::now();

    ros::Rate rate(100);
    while (ros::ok){
        blink();
        ros::spinOnce();
        rate.sleep();
    }
    
}