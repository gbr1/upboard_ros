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

int main(int argc, char **argv){
    ros::init(argc, argv, "leds_test_node");
    ros::NodeHandle nh;
    ros::Publisher led_pub = nh.advertise<upboard_ros::Led>("/upboard/leds", 10);

    upboard_ros::Led msg;
    bool s=false;
    while(ros::ok()){
        for(int i=0; i<10; i++){
            msg.header.stamp=ros::Time::now();
            msg.header.frame_id="base_link";
            msg.led=msg.ALL;
            msg.value=true;
            led_pub.publish(msg);
            ros::Duration(0.1).sleep();
            ros::spinOnce();
            msg.header.stamp=ros::Time::now();
            msg.header.frame_id="base_link";
            msg.led=msg.ALL;
            msg.value=false;
            led_pub.publish(msg);
            ros::Duration(0.5).sleep();
            ros::spinOnce();
        }
        for(int i=0; i<10; i++){
            //blue on
            msg.header.stamp=ros::Time::now();
            msg.header.frame_id="base_link";
            msg.led=msg.BLUE;
            msg.value=true;
            led_pub.publish(msg);
            ros::Duration(0.1).sleep();
            ros::spinOnce();
            //blue off
            msg.header.stamp=ros::Time::now();
            msg.header.frame_id="base_link";
            msg.led=msg.BLUE;
            msg.value=false;
            led_pub.publish(msg);
            ros::spinOnce();
            //yellow on
            msg.header.stamp=ros::Time::now();
            msg.header.frame_id="base_link";
            msg.led=msg.YELLOW;
            msg.value=true;
            led_pub.publish(msg);
            ros::Duration(0.1).sleep();
            ros::spinOnce();
            //yellow off
            msg.header.stamp=ros::Time::now();
            msg.header.frame_id="base_link";
            msg.led=msg.YELLOW;
            msg.value=false;
            led_pub.publish(msg);
            ros::spinOnce();
            //green on
            msg.header.stamp=ros::Time::now();
            msg.header.frame_id="base_link";
            msg.led=msg.GREEN;
            msg.value=true;
            led_pub.publish(msg);
            ros::Duration(0.1).sleep();
            ros::spinOnce();
            //green off
            msg.header.stamp=ros::Time::now();
            msg.header.frame_id="base_link";
            msg.led=msg.GREEN;
            msg.value=false;
            led_pub.publish(msg);
            ros::spinOnce();
            //red on
            msg.header.stamp=ros::Time::now();
            msg.header.frame_id="base_link";
            msg.led=msg.RED;
            msg.value=true;
            led_pub.publish(msg);
            ros::Duration(0.1).sleep();
            ros::spinOnce();
            //red off
            msg.header.stamp=ros::Time::now();
            msg.header.frame_id="base_link";
            msg.led=msg.RED;
            msg.value=false;
            led_pub.publish(msg);
            ros::spinOnce();
            //green on
            msg.header.stamp=ros::Time::now();
            msg.header.frame_id="base_link";
            msg.led=msg.GREEN;
            msg.value=true;
            led_pub.publish(msg);
            ros::Duration(0.1).sleep();
            ros::spinOnce();
             //green off
            msg.header.stamp=ros::Time::now();
            msg.header.frame_id="base_link";
            msg.led=msg.GREEN;
            msg.value=false;
            led_pub.publish(msg);
            ros::spinOnce();
            //yellow on
            msg.header.stamp=ros::Time::now();
            msg.header.frame_id="base_link";
            msg.led=msg.YELLOW;
            msg.value=true;
            led_pub.publish(msg);
            ros::Duration(0.1).sleep();
            ros::spinOnce();
            //yellow off
            msg.header.stamp=ros::Time::now();
            msg.header.frame_id="base_link";
            msg.led=msg.YELLOW;
            msg.value=false;
            led_pub.publish(msg);
            ros::spinOnce();
        }

    }

    return 0;
}