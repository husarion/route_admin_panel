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

Install dependencies:

```bash 
cd ~/ros_workspace/src/route_admin_panel/nodejs
npm install rosnodejs express socket.io quaternion-to-euler math3d multer yargs
npm install
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

- for ROSbot 2.0 with [Mbed firmware](https://github.com/husarion/rosbot-firmware-new):
    
    ```bash
    roslaunch route_admin_panel demo_rosbot_mbed_fw.launch
    ```
    
- for ROSbot 2.0 PRO with [Mbed firmware](https://github.com/husarion/rosbot-firmware-new):

    ```bash
    roslaunch route_admin_panel demo_rosbot_pro_mbed_fw.launch
    ```

- for Gazebo simulator:

    ```bash
    roslaunch route_admin_panel demo_gazebo.launch
    ```

- [deprecated] for ROSbot 2.0 with hFramework:

    ```bash
    roslaunch route_admin_panel demo_rosbot.launch
    ```

- [deprecated] for ROSbot 2.0 PRO with hFramework:

    ```bash
    roslaunch route_admin_panel demo_rosbot_pro.launch
    ```

Once all nodes are running, go to web browser and type in address bar:

```bash
ROSBOT_IP_ADDRESS:8000
```
You need to substitute phrase `ROSBOT_IP_ADDRESS` with IP address of your device.

You should see interface like below:

![RouteAdminPanelScreenshot](images/route-admin-panel.png)

## ROS API

Below are ROS interfaces used by the route admin panel:

### Topics

| Topic | Message type | Direction |&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Description&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|
| --- | --- | --- | --- |
| `/tf` | `tf2_msgs/TFMessage` | subscriber | Transform from `map` to `base_link` frame. |
| `/map_image/full/compressed` | `sensor_msgs/CompressedImage` | subscriber | Map converted to grayscale image and compressed in PNG format. |
| `/map_metadata` | `nav_msgs/MapMetaData` | subscriber | Metadata for map. |
| `/map_zoom` | `std_msgs/Int16` | publisher | Current value of map zoom. |
| `/plan` | `nav_msgs/Path` | publisher | Currently analyzed path from `make_plan` service. |


### Services

| Service name | Service type | Role |&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Description&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|
| --- | --- | --- | --- |
| `/move_base/make_plan` | `nav_msgs_service/GetPlan` | client | Get proposed path for given points without sending goal to navigation stack. |

### Actions

| Action name | Action type | Role |&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Description&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|
| --- | --- | --- | --- |
| `/move_base` | `move_base_msgs/MoveBase` | client | Set destinations for navigation stack. |

### Map to image conversion

RAP is using additional node `map_to_img_node` for conversion from `nav_msgs/OccupancyGrid` to `sensor_msgs/CompressedImage` from [`husarion_ros` package](https://github.com/husarion/husarion_ros). The `map_to_img_node` subscribes map as `nav_msgs/OccupancyGrid`  and publishes it as grayscale image with full resolution or cropped dependeing on current map zoom.

| Topic | Message type | Direction |&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Description&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|
| --- | --- | --- | --- |
| `/map` | `nav_msgs/OccupancyGrid` | subscriber | Map source |
| `/map_zoom` | `std_msgs/Int16` | subscriber | Current value of map zoom. |
| `/map_metadata` | `nav_msgs/MapMetaData` | publisher | Metadata for map. |
| `/map_image/full` | `sensor_msgs/Image` | publisher | Map converted to grayscale image. |
| `/map_image/tile` | `sensor_msgs/Image` | publisher | Map cropped according to zoom and converted to grayscale image. |

## Using panel from any network

In case you would like to manage robot destinations outside of local network, you could use [Husarnet](https://husarnet.com/) for secure connection with your robot.

All Husarion devices comes with Husarnet preisntalled, if you are using your own device, install Husarnet according to [installation guide](https://docs.husarnet.com/install/).

If you do not have a Husarnet account, create it and log in to [Husarnet dashboard](https://app.husarnet.com/).

In Husarnet dashboard, click **Create network** button, you will get a dialog:

![create network](images/husarnet_01_create-network.png)

Type `route_admin_demo` as network name then click **Create** button.

Go to your device and register it in Husarnet network by executing in terminal:

```
sudo husarnet websetup
```

You will get a registration link as a response, open it in web browser:

![add device](images/husarnet_02_husarnet.png)

- In **Name for this device** provide `my-rosbot`
- In **Add to network** dropdown menu choose `route_admin_demo`
- Check **Change device hostname** checkbox
- Click **Add device to your account** button

You will be redirected to network summary view:

![network summary](images/husarnet_03_network.png)

Click device name to open its configuration:

![network member](images/husarnet_04_network_member.png)

Check **ROS master** checkbox.

Optionally you can also check **Make the Web UI public** if you want to make panel accessible for anyone knowing device address.

Go back to your device and start panel with the same launch file as for local network.

Once the panel is running, you will notice new button **WebUI** next to your device address in Husarnet dashboard, use this button to view panel.

![web ui accessible](images/husarnet_05_network.png)

***Wait! But what about real peer-to-peer connection?***

To get access without need to log into any server, you will have to install Husarnet client also on your laptop, procedure is the same as for any other device.

Then register your laptop in Husarnet network the same way as you did with robot.

On laptop open browser and in address bar type: `[ROSBOT_HUSARNET_ADDRESS]:8000`
`ROSBOT_HUSARNET_ADDRESS` ia a value that you can find in Husarnet dashboard in device settings.

In the end you will be able to access `route_admin_panel` from any network using a secure peer-to-peer connection:

![panel accessed through husarnet](images/panel_at_husarnet.png)