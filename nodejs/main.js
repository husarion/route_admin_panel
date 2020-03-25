'use strict';

var app = require('express')();
var express = require('express');
var http = require('http').createServer(app);
var io = require('socket.io')(http);
const rclnodejs = require('rclnodejs');
var quaternionToEuler = require('quaternion-to-euler');
var math3d = require('math3d');
const fs = require('fs');
const yargs = require('yargs');
const uuidv1 = require('uuid/v1');

const NavTargets = require('./nav_targets.js');
const TfListener = require('./tf_listener.js');

const std_msgs = rclnodejs.require('std_msgs').msg;
const nav_msgs = rclnodejs.require('nav_msgs').msg;
const nav2_msgs = rclnodejs.require('nav2_msgs').action;
const geometry_msgs = rclnodejs.require('geometry_msgs').msg;
const nav_msgs_service = rclnodejs.require('nav_msgs').srv;
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

var nav2_actionClient;

var serviceClient;
var plan_publisher;
var current_plan;

var routePlanner = new RoutePlanner.RoutePlanner();
var route_active = false;

const argv = yargs
    .option('map_scale_min', {
        description: 'Minimal map scale',
        default: 5,
        alias: 'min',
        type: 'number'
    })
    .option('map_scale_max', {
        alias: 'max',
        default: 100,
        description: 'Maximal map scale',
        type: 'number',
    })
    .help()
    .alias('help', 'h')
    .version(false)
    .argv;

console.log("Map scale: [", argv.map_scale_min, ", ", argv.map_scale_max, "]")

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
    if (map_metadata && map_data_blob) {
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
    res.sendFile(__dirname + '/public/index.html');
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

function emit_route_status_update(label, sequence, seq_max) {
    let status = {
        seq: sequence,
        max: seq_max,
        label: label
    }
    io.emit('route_status', status);
}

function emit_current_path(path) {
    let path_object = {
        path: path
    }
    io.emit('current_plan', path_object);
}

function drive_to_target(navID) {
    let target = targets.get_target_by_id(navID);
    let action_goal = new nav2_msgs.NavigateToPose.Goal();
    let arr = new Array();
    let target_quaternion = math3d.Quaternion.Euler(0, 0, target.theta * 180 / Math.PI);
    uuidv1(null, arr, 0);
    action_goal._refObject.goal_id.uuid = arr;
    action_goal._refObject.pose.header.frame_id.data = "map";
    action_goal._refObject.pose.header.frame_id.size = 4;
    action_goal._refObject.pose.header.frame_id.capacity = 4;
    action_goal._refObject.pose.header.stamp.sec = Date.now() / 1000;
    action_goal._refObject.pose.pose.position.x = target.x;
    action_goal._refObject.pose.pose.position.y = target.y;
    action_goal._refObject.pose.pose.orientation.x = target_quaternion.x;
    action_goal._refObject.pose.pose.orientation.y = target_quaternion.y;
    action_goal._refObject.pose.pose.orientation.z = target_quaternion.z;
    action_goal._refObject.pose.pose.orientation.w = target_quaternion.w;
    let sequnece_number = nav2_actionClient.sendGoal(action_goal._refObject);
    nav2_actionClient._goal_uuid = arr;
    return sequnece_number;
}

io.on('connection', function (socket) {
    console.log('a user connected');
    socket.on('disconnect', function () {
        console.log('user disconnected');
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
        drive_to_target(targetID);
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

    socket.on('route_start', function (mode) {
        console.log("Start route in " + mode + " mode");
        routePlanner.sequenceMode = mode;
        route_active = true;
        let goalNavID = routePlanner.getNextGoal();
        let goal_sequence_number = drive_to_target(goalNavID);
        let routeID;
        let sequence;
        let goalLabel;
        for (let i = 0; i < routePlanner.goalList.length; i++) {
            if (routePlanner.goalList[i].getNavID() == goalNavID) {
                routeID = routePlanner.goalList[i].getRouteID();
                sequence = i;
                goalLabel = targets.get_target_by_id(goalNavID).label;
            }
        }
        routePlanner.goalAccepted(routeID, goal_sequence_number);
        emit_route_status_update(goalLabel, sequence, routePlanner.goalList.length - 1);
    });

    socket.on('route_stop', function () {
        console.log("Stop route");
        route_active = false;
    });

    socket.on('make_plan', function (points) {
        let startTarget = targets.get_target_by_id(Number(points.start));
        let endTarget = targets.get_target_by_id(Number(points.end));

        let start_point = new geometry_msgs.PoseStamped();
        let start_quaternion = math3d.Quaternion.Euler(0, 0, startTarget.theta * 180 / Math.PI);
        start_point.header.frame_id = "map";
        start_point.header.stamp.secs = Date.now() / 1000;
        start_point.pose.position.x = startTarget.x;
        start_point.pose.position.y = startTarget.y;
        start_point.pose.position.z = 0;
        start_point.pose.orientation.x = start_quaternion.x;
        start_point.pose.orientation.y = start_quaternion.y;
        start_point.pose.orientation.z = start_quaternion.z;
        start_point.pose.orientation.w = start_quaternion.w;

        let end_point = new geometry_msgs.PoseStamped();
        let end_quaternion = math3d.Quaternion.Euler(0, 0, endTarget.theta * 180 / Math.PI);
        end_point.header.frame_id = "map";
        end_point.header.stamp.secs = Date.now() / 1000;
        end_point.pose.position.x = endTarget.x;
        end_point.pose.position.y = endTarget.y;
        end_point.pose.position.z = 0;
        end_point.pose.orientation.x = end_quaternion.x;
        end_point.pose.orientation.y = end_quaternion.y;
        end_point.pose.orientation.z = end_quaternion.z;
        end_point.pose.orientation.w = end_quaternion.w;

        let req = new nav_msgs_service.GetPlan.Request();
        req.start = start_point;
        req.goal = end_point;

        serviceClient.call(req)
            .then((resp) => {
                current_plan = resp.plan;
                current_plan.header.frame_id = '/map';
                plan_publisher.publish(current_plan);
                emit_current_path(current_plan);
            });
    })

    let scale_range = {
        min: argv.map_scale_min,
        max: argv.map_scale_max
    }
    io.emit('set_scale_range', scale_range);

    targets.targets.forEach(emit_target);
    for (let i = 0; i < routePlanner.goalList.length; i++) {
        emit_route_point(i, routePlanner.goalList[i].getNavID(), routePlanner.goalList[i].getRouteID());
    }

    emit_map_update();
});

load_config();

http.listen(8000, function () {
    console.log('listening on *:8000');
});

rclnodejs.init().then(() => {
    const rosNode = rclnodejs.createNode('rap_server_node');
    robot_pose_emit_timestamp = Date.now();

    rosNode.createSubscription('tf2_msgs/msg/TFMessage', '/tf',
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

    rosNode.createSubscription('nav_msgs/msg/MapMetaData', '/map_metadata',
        (data) => {
            map_metadata = data;
        }, {
        queueSize: 1,
        throttleMs: 0
    }
    );

    rosNode.createSubscription('sensor_msgs/msg/CompressedImage', '/map_image/full/compressed',
        {
            queueSize: 1,
            throttleMs: 0
        },
        (data) => {
            map_data_blob = data.data;
            emit_map_update();
        }
    );

    // serviceClient = nh.serviceClient('/move_base/make_plan', nav_msgs_service.GetPlan);

    plan_publisher = rosNode.createPublisher(nav_msgs.Path, '/plan');

    nav2_actionClient = rosNode.createActionClient(
        'nav2_msgs/action/NavigateToPose',
        'NavigateToPose',
        {
            queueSize: 1,
            throttleMs: 1
        },
        (data) => {
            data.status_list.forEach(status => {
                let status_id = new Array();
                status.goal_info.goal_id.uuid.forEach((byte) => {
                    status_id.push(byte);
                })
                if (JSON.stringify(status_id) === JSON.stringify(nav2_actionClient._goal_uuid)) {
                    if (nav2_actionClient.GOAL_STATES[status.status - 1] == 'SUCCEEDED') {
                        if (route_active) {
                            switch (routePlanner.sequenceMode) {
                                case RoutePlanner.SequenceModes.LOOP_RUN:
                                    break;
                                case RoutePlanner.SequenceModes.SINGLE_RUN:
                                    break;
                                case RoutePlanner.SequenceModes.BACK_AND_FORTH:
                                    break;
                            }
                            let goalNavID = routePlanner.getNextGoal();
                            if (!goalNavID) {
                                console.log("End of route");
                                route_active = false;
                            } else {
                                let goal_sequence_number = drive_to_target(goalNavID);
                                let routeID;
                                let sequence;
                                let goalLabel;
                                for (let i = 0; i < routePlanner.goalList.length; i++) {
                                    if (routePlanner.goalList[i].getNavID() == goalNavID) {
                                        routeID = routePlanner.goalList[i].getRouteID();
                                        sequence = i;
                                        goalLabel = targets.get_target_by_id(goalNavID).label;
                                    }
                                }
                                routePlanner.goalAccepted(routeID, goal_sequence_number);
                                emit_route_status_update(goalLabel, sequence, routePlanner.goalList.length - 1);
                            }

                        } else {
                            emit_route_status_update("Finished", 0, 0);
                        }
                    }
                    else {
                        emit_route_status_update("CANCELLED", 0, 0);
                    }
                }
            })
        },
        (data) => {
            console.log("Received feedback: ", data);
        },
        (data) => {
            console.log("Received goal response: ", data);
        },
        (data) => {
            console.log("Received result response: ", data);
        }
    );

    rclnodejs.spin(rosNode);
})
    .catch((err) => {
        console.log("rclodejs error");
        console.log(err);
    });