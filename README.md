## About

The **Route admin panel** is a web user interface for managing routes of ROS based mobile robots.

It allows to:
- define destination points
- save robot position as destination point
- send destination point to `move_base`
- upload custom map
- set a sequence of destination points

Web server for ROS <==> Browser communication is based on [Node.js](https://nodejs.org/).

## Installation

Install [Node.js](https://nodejs.org/):

```
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
sudo apt install -y nodejs
```

Clone and build Husarion fork of `rosnodejs` repository:
```
mkdir ~/husarion_rosnodejs
cd ~/husarion_rosnodejs
git clone https://github.com/lukaszmitka/rosnodejs.git
cd ~/husarion_rosnodejs/rosnodejs
npm install
npm pack
```

Create workspace and clone dependency repositories, it may happen that you already have it done, in that case, skip this step:
```
mkdir ~/ros_workspace
mkdir ~/ros_workspace/src
cd ~/ros_workspace/src
catkin_init_workspace 
echo '. ~/ros_workspace/devel/setup.sh' >> ~/.bashrc

git clone https://github.com/husarion/husarion_ros.git
git clone https://github.com/husarion/rosbot_description.git
```

Clone `route_admin_panel` repository:
```
cd ~/ros_workspace/src
git clone https://github.com/husarion/route_admin_panel.git
```

Install rosnodejs and dependencies:
```
cp ~/husarion_rosnodejs/rosnodejs/rosnodejs-3.0.0.tgz ~/ros_workspace/src/route_admin_panel/nodejs
cd ~/ros_workspace/src/route_admin_panel/nodejs/
npm install rosnodejs-3.0.0.tgz
npm install express socket.io quaternion-to-euler math3d multer
npm install
mkdir user_maps
echo '{"targetList": {"targets": []}}' > user_maps/config.json
```

Build workspace:
```
cd ~/ros_workspace
catkin_make
. ~/ros_workspace/devel/setup.sh
```

## How to use

Start using launch file, depending on your ROSbot version:


- for ROSbot 2.0:
    ```
    roslaunch route_admin_panel demo_rosbot.launch
    ```

- for ROSbot 2.0 PRO:

    ```
    roslaunch route_admin_panel demo_rosbot_pro.launch
    ```
- for Gazebo simulator:
    ```
    roslaunch route_admin_panel demo_gazebo.launch
    ```

Open panel in browser by typing:
```
rosbot_IP:3000
```
