## About

The **Route admin panel** is a web user interface for managing routes of ROS based mobile robots.

It allows to:
- Define destination points
- Save robot position as destination point
- Send destination point to `move_base`
- Upload custom map
- Set a sequence of destination points

The **Route admin panel** is built as a [Node.js](https://nodejs.org/) application. On one side it is interfacing with ROS topics, while on another side it presents a frontend for managing robot destinations.

## Installation

Install [Node.js](https://nodejs.org/):

```bash
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
sudo apt install -y nodejs
```

Clone and build Husarion fork of `rosnodejs` repository:

```bash
mkdir ~/husarion_rosnodejs
cd ~/husarion_rosnodejs
git clone https://github.com/RethinkRobotics-opensource/rosnodejs.git
cd ~/husarion_rosnodejs/rosnodejs
npm install
npm pack
```

Create workspace and clone dependency repositories, it may happen that you already have it done, in that case, skip this step:

```bash
mkdir ~/ros_workspace
mkdir ~/ros_workspace/src
cd ~/ros_workspace/src
catkin_init_workspace 
echo '. ~/ros_workspace/devel/setup.sh' >> ~/.bashrc

git clone https://github.com/husarion/husarion_ros.git
git clone https://github.com/husarion/rosbot_description.git
```

Clone `route_admin_panel` repository:

```bash
cd ~/ros_workspace/src
git clone https://github.com/husarion/route_admin_panel.git
```

Install rosnodejs and dependencies:

```bash 
cp ~/husarion_rosnodejs/rosnodejs/rosnodejs-3.0.0.tgz ~/ros_workspace/src/route_admin_panel/nodejs
cd ~/ros_workspace/src/route_admin_panel/nodejs/
npm install rosnodejs-3.0.0.tgz
npm install express socket.io quaternion-to-euler math3d multer
npm install
mkdir user_maps
echo '{"targetList": {"targets": []}}' > user_maps/config.json
```

Build workspace:

```bash
cd ~/ros_workspace
catkin_make
. ~/ros_workspace/devel/setup.sh
```

## How to use

Panel comes with prepared launch files for `move_base`, `gmapping`, `node.js` server and all other required components.
Depending on your ROSbot version, you can start it with:

- for ROSbot 2.0:

    ```bash
    roslaunch route_admin_panel demo_rosbot.launch
    ```

- for ROSbot 2.0 PRO:

    ```bash
    roslaunch route_admin_panel demo_rosbot_pro.launch
    ```
- for Gazebo simulator:

    ```bash
    roslaunch route_admin_panel demo_gazebo.launch
    ```

Once all nodes are running, go to web browser and type in address bar:

```bash
ROSBOT_IP_ADDRESS:3000
```
You need to substitute phrase `ROSBOT_IP_ADDRESS` with IP address of your device.

You should see interface like below:

![RouteAdminPanelScreenshot](images/route-admin-panel.png)

