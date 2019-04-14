# upboard_ros
**This package is compatible with [UpBoard](https://up-board.org/up/specifications/), [UpSquared](https://up-board.org/upsquared/specifications/) and [UpXtreme](https://up-board.org/up-xtreme/)**<br>
It is part of [Erwhi Hedgehog](https://gbr1.github.io/erwhi_hedgehog.html) development.

---
## 1. Setup
```
cd ~/catkin_ws/src
git clone https://github.com/gbr1/upboard_ros.git
cd ..
catkin_make
catkin_make install
```
## 2. Test
Leds animations:<br>
`roslauch upboard_ros led_test.launch`<br>
<br>
Manual test:
1. Open a terminal:<br>
`roscore`
2. In a new terminal:<br>
`rosrun upboard_ros led_node`<br>
3. In a new terminal:<br>
to turn on all leds:<br>
`rostopic pub /upboard/leds upboard_ros/Leds "{header: auto, leds:[led: 0, value: 1]}" --once`<br>
to turn off all leds:<br>
`rostopic pub /upboard/leds upboard_ros/Leds "{header: auto, leds:[led: 0, value: 0]}" --once`<br>
## 3. Nodes
3.1 ***led_node***, allows you to connect to upboard leds
- **Subcribed**:
    - ***/upboard/leds***<br>
        Header header<br>
        Led[] leds<br>
        _Led:_<br>
        _uint8 led, you can use also constants ALL=0, BLUE=1, YELLOW=2, GREEN=3, RED=4_<br>
        _bool value, true or 1 is on and false or 0 is off_

3.2 ***led_test***, allows you to test upboard leds
- **Published**:
    - _
    ***/upboard/leds***<br>



## 4. Future
Idea is to add also GPIO and other useful data from Upboard.
<br>
<br>


***Copyright (c) 2019 Giovanni di Dio Bruno under MIT license***
