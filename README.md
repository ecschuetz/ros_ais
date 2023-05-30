# ros_ais


**Author**: Enrico Schuetz

Software License Agreement (BSD License)

**Description**: ROS driver for the (Automatic Identification System)
* CLASS A AIS POSITION REPORT (MESSAGES 1, 2, AND 3)
* AIS BASE STATION REPORT (MESSAGE 4) AND COORDINATED UNIVERSAL TIME AND DATE RESPONSE (MESSAGE 11)
* MESSAGE 8 DAC 200 . geometry report

## 1. tested ROS distribution

* ROS2 Hubmle (Ubuntu 22.04)

## 2. Dependencies
* Python 3
* AIS receiver
* pyserial
* libais


## 3. Installation



pyserial must be installed if it is not on the computer.

```
$ pip install pyserial
```
install libais for `import ais`
```
git clone https://github.com/schwehr/libais.git
cd libais
python3 setup.py build
python3 setup.py install
```
create ros package
```
mkdir -p ~/<$ROS_WORKSPACE>/src
cd ~/<$ROS_WORKSPACE>/src
git clone -b ros2 https://github.com/ecschuetz/ros_ais.git
git clone https://github.com/ecschuetz/ais_common.git
cd ..
colcon build
source install/setup.bash
```

## 4. launch ais

edit launch file `launch/ais.launch.py` for correct serial port and baudrate
```
ros2 launch ros_ais ais.launch.py
```


## 5. mentioned reports 
in `msg/rosais123.msg` you will find the class A AIS Position Report Messages, as defined in libais.
For more information, see [https://gpsd.gitlab.io/gpsd/AIVDM.html](https://gpsd.gitlab.io/gpsd/AIVDM.html) and [https://github.com/schwehr/libais/blob/master/src/libais/ais1_2_3.cpp](https://github.com/schwehr/libais/blob/master/src/libais/ais1_2_3.cpp)

in `msg/rosais411.msg` you find the class A AIS Base Station Report.
