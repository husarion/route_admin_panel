'use strict';

var app = require('express')();
var express = require('express');
var http = require('http').createServer(app);
var io = require('socket.io')(http);
const rosnodejs = require('rosnodejs');
var quaternionToEuler = require('quaternion-to-euler');
var math3d = require('math3d');
const fs = require('fs');

const NavTargets = require('./nav_targets.js');
const TfListener = require('./tf_listener.js');

const std_msgs = rosnodejs.require('std_msgs').msg;
const nav_msgs = rosnodejs.require('nav_msgs').msg;
const geometry_msgs = rosnodejs.require('geometry_msgs').msg;
const move_base_msgs = rosnodejs.require('move_base_msgs').msg;
const RoutePlanner = require('./route_planner.js');

var multer = require('multer');

const { exec } = require('child_process');

var storage = multer.diskStorage({
    destination: function (req, file, cb) {
        cb(null, './user_maps/');
    },
    filename: function (req, file, cb) {
        cb(null, Date.now() + file.originalname);
    }
});

var upload = multer({ storage: storage });

var zoom_publisher;

const zoom_msg = new std_msgs.Int16();
const drive_msg = new geometry_msgs.Twist();
var cmd_vel_publisher;

var targets = new NavTargets.TargetList();

var map_data_blob;
var map_metadata;

var map_server_process;
var custom_map_file;
var configFileName = './user_maps/config.json';

var tfTree = new TfListener.TfTree();
var robot_pose_emit_timestamp;

var moveBase_actionClient

var routePlanner = new RoutePlanner.RoutePlanner();

function save_config() {
    let confObject = {
        customMapFile: custom_map_file,
        targetList: targets
    }
    let jsonString = JSON.stringify(confObject);
    fs.writeFile(configFileName, jsonString, 'utf8', function (err) {
        if (err) {
            console.log(err);
        }
    });
}

function load_config() {
    fs.readFile(configFileName, 'utf8', function (err, data) {
        if (err) {
            console.log(err);
        } else {
            let confObject = JSON.parse(data);
            targets.targets = confObject.targetList.targets;
            if (confObject.customMapFile) {
                custom_map_file = confObject.customMapFile;
                startMapServer(custom_map_file);
            }
        }
    });
}

const default_publisher_options = {
    queueSize: 1,
    latching: false,
    throttleMs: 100
}

function emit_map_update() {
    if (map_metadata) {
        let map_update = {
            blob: map_data_blob,
            metadata: map_metadata
        }
        io.emit('map_update', map_update);
    }
}

function emit_robot_pose() {
    if (robot_pose_emit_timestamp + 50 < Date.now()) {
        robot_pose_emit_timestamp = Date.now();
        let transform = tfTree.lookup_transform('map', 'base_link', 0);
        if (transform) {
            var euler_angles = quaternionToEuler([
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w
            ]);
            var pose_msg = {
                x_pos: transform.translation.x,
                y_pos: transform.translation.y,
                theta: euler_angles[0]
            };
            io.emit('robot_pose', pose_msg);
        }
    }
}

function killMapServer() {
    if (map_server_process) {
        map_server_process.kill();
    }
}

function startMapServer(map_file) {
    killMapServer();
    console.log("start map server with: " + map_file);
    map_server_process = exec('rosrun map_server map_server ' + map_file, (err, stdout, stderr) => {
        console.log("Subprocess finished");
        if (err) {
            console.log("Error: " + err);
            return;
        }
    });
    console.log("Subprocess started");
}

app.get('/', function (req, res) {
    res.sendFile(__dirname + '/index.html');
});

app.use(express.static('public'))

app.post('/upload', upload.single('map-image'), function (req, res) {
    custom_map_file = "./user_maps/user_map.yaml";
    let imagePath = req.file.path.replace(/^user_maps\//, '');
    let yaml_ouptut = 'image: ' + imagePath + '\n';
    yaml_ouptut += 'resolution: ' + req.body.mapResolution + '\n';
    yaml_ouptut += 'origin: [0.0, 0.0, 0.0]\n';
    yaml_ouptut += 'occupied_thresh: 0.65\n';
    yaml_ouptut += 'free_thresh: 0.196\n';
    yaml_ouptut += 'negate: 0\n';
    fs.writeFile(custom_map_file, yaml_ouptut, function (err) {
        if (err) {
            return console.log(err);
        }
    });
    save_config();
    startMapServer(custom_map_file);
    res.end();
});

function emit_target(target) {
    io.emit('add_target', target);
}

function emit_target_delete(target_id) {
    io.emit('remove_target_by_id', target_id);
}

function emit_route_point(sequence, navID, routeID) {
    let route_point = {
        route_sequence: sequence,
        point_navID: navID,
        point_routeID: routeID,
        target: targets.get_target_by_id(navID)
    }
    io.emit('new_route_point', route_point);
}

function emit_del_route_point(routeID) {
    io.emit('del_route_point', routeID);
}

io.on('connection', function (socket) {
    console.log('a user connected');
    socket.on('disconnect', function () {
        console.log('user disconnected');
    });

    socket.on('map_scale', function (map_scale) {
        zoom_msg.data = map_scale;
        zoom_publisher.publish(zoom_msg);
    });

    socket.on('drive_command', function (drive_command) {
        drive_msg.linear.x = drive_command.lin;
        drive_msg.angular.z = drive_command.ang;
        cmd_vel_publisher.publish(drive_msg);
    });

    socket.on('delete_target', function (target_id) {
        targets.remove_target(target_id);
        emit_target_delete(target_id);
        save_config();
    });

    socket.on('new_target', function (new_target) {
        console.log("New target\n", new_target);
        let target = new NavTargets.Target(targets.get_next_id(), new_target.x, new_target.y, new_target.theta, new_target.label);
        targets.add_target(target);
        emit_target(target);
        save_config();
    });

    socket.on('drive_to_target', function (targetID) {
        let target = targets.get_target_by_id(targetID);
        let move_base_goal = new move_base_msgs.MoveBaseGoal();
        let set_point = new geometry_msgs.PoseStamped();
        let target_quaternion = math3d.Quaternion.Euler(0, 0, target.theta * 180 / Math.PI);
        set_point.header.frame_id = "map";
        set_point.header.stamp.secs = Date.now() / 1000;
        set_point.pose.position.x = target.x;
        set_point.pose.position.y = target.y;
        set_point.pose.orientation.x = target_quaternion.x;
        set_point.pose.orientation.y = target_quaternion.y;
        set_point.pose.orientation.z = target_quaternion.z;
        set_point.pose.orientation.w = target_quaternion.w;
        move_base_goal.target_pose = set_point;
        let goal_handle = moveBase_actionClient.sendGoal(move_base_goal);
        console.log(goal_handle._goal.goal_id);
    });

    socket.on('add_to_route', function (navID) {
        let pointSummary = routePlanner.addGoal(navID);
        emit_route_point(pointSummary.sequence, pointSummary.navID, pointSummary.routeID);
    });

    socket.on('remove_from_route', function (routeID) {
        routePlanner.removeGoal(routeID);
        emit_del_route_point(routeID);
    });

    socket.on('move_up_on_route', function (routeID) {
        let pointSummary = routePlanner.moveGoalUp(routeID);
        emit_del_route_point(routeID);
        emit_route_point(pointSummary.sequence, pointSummary.navID, pointSummary.routeID);
    });

    socket.on('move_down_on_route', function (routeID) {
        let pointSummary = routePlanner.moveGoalDown(routeID);
        emit_del_route_point(routeID);
        emit_route_point(pointSummary.sequence, pointSummary.navID, pointSummary.routeID);
    });

    targets.targets.forEach(emit_target);
    emit_map_update();
});

load_config();

http.listen(3000, function () {
    console.log('listening on *:3000');
});

rosnodejs.initNode('/rosnodejs')
    .then((rosNode) => {
        robot_pose_emit_timestamp = Date.now();
        zoom_publisher = rosNode.advertise('/map_zoom', std_msgs.Int16, default_publisher_options);

        drive_msg.linear.x = 0;
        drive_msg.linear.y = 0;
        drive_msg.linear.z = 0;
        drive_msg.angular.x = 0;
        drive_msg.angular.y = 0;
        drive_msg.angular.z = 0;
        cmd_vel_publisher = rosNode.advertise('/cmd_vel', geometry_msgs.Twist, default_publisher_options);

        let pose_subscriber = rosNode.subscribe('/tf', 'tf2_msgs/TFMessage',
            (data) => {
                data.transforms.forEach(transform_stamped => {
                    if (!tfTree.frame_id) {
                        tfTree = new TfListener.TfTree(transform_stamped.header.frame_id);
                    }
                    let transform = new TfListener.TfTransform(transform_stamped.header.frame_id, transform_stamped.child_frame_id, transform_stamped.header.stamp, transform_stamped.transform);
                    tfTree.add_transform(transform, 0);
                });
                emit_robot_pose();
            }, {
                queueSize: 1,
                throttleMs: 0
            }
        );

        let map_metadata_subscriber = rosNode.subscribe('/map_metadata', 'nav_msgs/MapMetaData',
            (data) => {
                map_metadata = data;
            }, {
                queueSize: 1,
                throttleMs: 0
            }
        );

        let map_subscriber = rosNode.subscribe('/map_image/full/compressed', 'sensor_msgs/CompressedImage',
            (data) => {
                map_data_blob = data.data;
                emit_map_update();
            }, {
                queueSize: 1,
                throttleMs: 0
            }
        );

        const nh = rosnodejs.nh;
        moveBase_actionClient = new rosnodejs.ActionClient({
            nh,
            type: 'move_base_msgs/MoveBase',
            actionServer: '/move_base'
        });
        console.log("Subscribe to move_base updates");
        moveBase_actionClient.on('status', (data) => {
            // console.log("Received move_base: status");
            data.status_list.forEach(status => {
                // console.log("    Goal ID: " + status.goal_id.id + ", status: " + status.status);
            });
        });
        // moveBase_actionClient.on('feedback', (data) => {console.log("Received move_base: feedback\n", data);});
        // moveBase_actionClient.on('result', (data) => {console.log("Received move_base: result\n", data);});
    })
    .catch((err) => {
        rosnodejs.log.error(err.stack);
    });