'use strict';

var app = require('express')();
var express = require('express');
var http = require('http').createServer(app);
var io = require('socket.io')(http);
const rosnodejs = require('rosnodejs');
var quaternionToEuler = require('quaternion-to-euler');
var math3d = require('math3d');
const fs = require('fs');
const yargs = require('yargs');
var path = require('path');

const NavTargets = require('./nav_targets.js');
const TfListener = require('./tf_listener.js');

const std_msgs = rosnodejs.require('std_msgs').msg;
const nav_msgs = rosnodejs.require('nav_msgs').msg;
const geometry_msgs = rosnodejs.require('geometry_msgs').msg;
const move_base_msgs = rosnodejs.require('move_base_msgs').msg;
const nav_msgs_service = rosnodejs.require('nav_msgs').srv;
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
const initial_pose_msg = new geometry_msgs.PoseWithCovarianceStamped();
var cmd_vel_publisher;
var initialpose_publisher;

var targets = new NavTargets.TargetList();

var map_data_blob;
var map_metadata;

var map_server_process;
var amcl_process;
var global_localization_process;
var gmapping_process;
var autosave_interval;
var autosave_interval_time;
var custom_map_file;
var selected_map_mode;
var map_autosave;
var configDirectory = './user_maps';
var configFileName = './user_maps/config.json';

var tfTree = new TfListener.TfTree();
var robot_pose_emit_timestamp;

var moveBase_actionClient

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
    .option('map_autosave_interval', {
        alias: 'autosave',
        default: 60000,
        description: 'Interval for map autosaving, [ms]',
        type: 'number',
    })
    .option('port', {
        default: 8000,
        description: 'Port for server to listen on',
        type: 'number',
    })
    .help()
    .alias('help', 'h')
    .version(false)
    .argv;

console.log("Map scale: [", argv.map_scale_min, ", ", argv.map_scale_max, "]")
console.log("Port: ", argv.port)
console.log("Autosave interval time: ", argv.port, "[ms]")

autosave_interval_time = argv.map_autosave_interval;

function save_config() {
    let confObject = {
        mapMode: selected_map_mode,
        customMapFile: custom_map_file,
        autosaveEnable: map_autosave,
        targetList: targets
    }
    let jsonString = JSON.stringify(confObject);
    fs.writeFile(configFileName, jsonString, 'utf8', function (err) {
        if (err) {
            console.log(err);
        }
    });
    load_config();
}

function load_config() {
    if (fs.existsSync(configFileName)) {
        fs.readFile(configFileName, 'utf8', function (err, data) {
            if (err) {
                console.log(err);
            } else {
                let confObject = JSON.parse(data);
                targets.targets = confObject.targetList.targets;
                if (confObject.mapMode == 'SLAM') {
                    stopAMCL();
                    stopMapServer();
                    console.log("Start SLAM process");
                    startGmapping();
                    if (confObject.autosaveEnable == true) {
                        console.log("Start map autosave process");
                        startAutoSave();
                    } else {
                        stopAutoSave();
                    }
                } else if (confObject.mapMode == "STATIC") {
                    stopGmapping();
                    stopAutoSave();
                    custom_map_file = confObject.customMapFile;
                    startMapServer(custom_map_file);
                    startAMCL();
                }
            }
        });
    } else {
        console.log("Config file does not exist, create default.");
        if (!fs.existsSync(configDirectory)) {
            fs.mkdirSync(configDirectory);
        }
        selected_map_mode = 'SLAM';
        custom_map_file = '';
        map_autosave = true;
        save_config();
    }
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

function setInitialPose(initial_pose) {
    let target_quaternion = math3d.Quaternion.Euler(0, 0, initial_pose.robot_pos_theta * 180 / Math.PI);
    initial_pose_msg.header.frame_id = "map";
    initial_pose_msg.header.stamp = rosnodejs.Time.now();
    initial_pose_msg.pose.pose.position.x = initial_pose.robot_pos_x;
    initial_pose_msg.pose.pose.position.y = initial_pose.robot_pos_y;
    initial_pose_msg.pose.pose.position.z = 0;
    initial_pose_msg.pose.pose.orientation.x = target_quaternion.x;
    initial_pose_msg.pose.pose.orientation.y = target_quaternion.y;
    initial_pose_msg.pose.pose.orientation.z = target_quaternion.z;
    initial_pose_msg.pose.pose.orientation.w = target_quaternion.w;
    initialpose_publisher.publish(initial_pose_msg);
}

function stopMapServer() {
    if (map_server_process) {
        exec('rosnode kill static_map_server', (err, stdout, stderr) => {
            if (err) {
                console.log("Could not stop map server: " + err);
            }
        });
        map_server_process.kill();
    }
}

function startMapServer(map_file) {
    console.log("start map server with: " + map_file);
    map_server_process = exec('rosrun map_server map_server ' + configDirectory + '/' + map_file + ' __name:=static_map_server', (err, stdout, stderr) => {
        console.log("Subprocess finished");
        if (err) {
            console.log("Error: " + err);
            return;
        }
    });
    console.log("Subprocess started");
}

function stopAMCL() {
    if (amcl_process) {
        exec('rosnode kill amcl', (err, stdout, stderr) => {
            if (err) {
                console.log("Could not stop AMCL: " + err);
            }
        });
        amcl_process.kill();
    }
    if (global_localization_process) {
        exec('rosnode kill amcl_estimate_node', (err, stdout, stderr) => {
            if (err) {
                console.log("Could not stop AMCL estimate: " + err);
            }
        });
        global_localization_process.kill();
    }
}

function startAMCL() {
    console.log("Start AMCL");
    amcl_process = exec('roslaunch route_admin_panel amcl.launch', (err, stdout, stderr) => {
        console.log("AMCL finished");
        if (err) {
            console.log("Error: " + err);
            return;
        }
    });
    console.log("AMCL launched");
    global_localization_process = exec('roslaunch route_admin_panel amcl_estimate.launch', (err, stdout, stderr) => {
        console.log("AMCL estimate finished");
        if (err) {
            console.log("Error: " + err);
            return;
        }
    });
    console.log("AMCL estimate launched");
}

function stopGmapping() {
    if (gmapping_process) {
        exec('rosnode kill gmapping_node', (err, stdout, stderr) => {
            if (err) {
                console.log("Could not stop gmapping: " + err);
            }
        });
        gmapping_process.kill();
        gmapping_process = null;
    }
}

function startGmapping() {
    if (gmapping_process) {
        console.log("Gmapping is already running, no need to launch it again")
    } else {
        gmapping_process = exec('roslaunch route_admin_panel gmapping.launch', (err, stdout, stderr) => {
            console.log("Gmapping finished");
            if (err) {
                console.log("Error: " + err);
                return;
            }
            gmapping_process = null;
        });
        console.log("Gmapping launched");
    }
}

function stopAutoSave() {
    clearInterval(autosave_interval);
}

function startAutoSave() {
    stopAutoSave();
    let date_string = Date.now().toString();
    console.log("Enable map auto save");
    autosave_interval = setInterval(saveMap, autosave_interval_time, 'auto_saved_map_' + date_string);
}

function saveMap(filename) {
    console.log(`Saving map with name: ${filename}`);
    exec('cd ' + configDirectory + ' && rosrun map_server map_saver -f ' + filename, (err, stdout, stderr) => {
        if (err) {
            console.log("[map_server map_saver] Error: " + err);
            return;
        }
        console.log("[map_server map_saver] " + stdout);
        console.log("[map_server map_saver] " + stderr);
        update_map_filenames();
        console.log("[map_server map_saver] Process finished");
    });
}


app.get('/', function (req, res) {
    res.sendFile(__dirname + '/public/index.html');
});

app.use(express.static('public'))

app.post('/upload', upload.single('map-image'), function (req, res) {
    custom_map_file = './user_maps/' + req.file.originalname + '.yaml';
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
    update_map_filenames();
    res.end();
});

function save_map_settings(settings) {
    console.log("Received map settings");
    console.log(settings);
    if (settings.map_static == true) {
        selected_map_mode = 'STATIC';
    } else if (settings.map_slam == true) {
        selected_map_mode = 'SLAM';
    }
    custom_map_file = settings.map_file + '.yaml';
    map_autosave = settings.map_autosave;
    save_config();
}

function update_map_filenames() {
    let filenames = getFileList('./user_maps', '.yaml');
    io.emit('map_file_list', filenames);
}

function getFileList(startPath, filter) {

    console.log('Starting from dir ' + startPath + '/');

    if (!fs.existsSync(startPath)) {
        console.log("no dir ", startPath);
        return;
    }
    let file_names = [];
    var files = fs.readdirSync(startPath);
    for (var i = 0; i < files.length; i++) {
        var filename = path.join(startPath, files[i]);
        var stat = fs.lstatSync(filename);
        if (stat.isDirectory()) {
            getFileList(filename, filter); //recurse
        }
        else if (filename.indexOf(filter) >= 0) {
            file_names.push(filename.substring(10, filename.length - 5));
        };
    };
    return file_names;
};

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
    let move_base_goal = new move_base_msgs.MoveBaseGoal();
    let set_point = new geometry_msgs.PoseStamped();
    let target_quaternion = math3d.Quaternion.Euler(0, 0, target.theta * 180 / Math.PI);
    set_point.header.frame_id = "map";
    set_point.header.stamp = rosnodejs.Time.now();
    set_point.pose.position.x = target.x;
    set_point.pose.position.y = target.y;
    set_point.pose.orientation.x = target_quaternion.x;
    set_point.pose.orientation.y = target_quaternion.y;
    set_point.pose.orientation.z = target_quaternion.z;
    set_point.pose.orientation.w = target_quaternion.w;
    move_base_goal.target_pose = set_point;
    let goal_handle = moveBase_actionClient.sendGoal(move_base_goal);
    console.log(goal_handle._goal.goal_id);
    return goal_handle._goal.goal_id;
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
        let actionGoalID = drive_to_target(goalNavID);
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
        routePlanner.goalAccepted(routeID, actionGoalID);
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

    socket.on('save_map_settings', save_map_settings);

    let scale_range = {
        min: argv.map_scale_min,
        max: argv.map_scale_max
    }
    io.emit('set_scale_range', scale_range);

    targets.targets.forEach(emit_target);
    for (let i = 0; i < routePlanner.goalList.length; i++) {
        emit_route_point(i, routePlanner.goalList[i].getNavID(), routePlanner.goalList[i].getRouteID());
    }

    socket.on('set_initialpose', setInitialPose);

    emit_map_update();
    update_map_filenames();
});

load_config();

http.listen(argv.port, function () {
    console.log('listening on *:' + argv.port);
});

rosnodejs.initNode('/rosnodejs')
    .then((rosNode) => {
        robot_pose_emit_timestamp = Date.now();
        zoom_publisher = rosNode.advertise('/map_zoom', std_msgs.Int16, default_publisher_options);

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

        serviceClient = nh.serviceClient('/move_base/make_plan', nav_msgs_service.GetPlan);

        plan_publisher = rosNode.advertise('/plan', nav_msgs.Path, default_publisher_options);

        initialpose_publisher = rosNode.advertise('/initialpose', geometry_msgs.PoseWithCovarianceStamped, default_publisher_options);

        moveBase_actionClient._acInterface.on('result', (data) => {
            console.log("Received move_base: result\n", data);
            if (data.status.status == 3) {
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
                        let actionGoalID = drive_to_target(goalNavID);
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
                        routePlanner.goalAccepted(routeID, actionGoalID);
                        emit_route_status_update(goalLabel, sequence, routePlanner.goalList.length - 1);
                    }

                } else {
                    emit_route_status_update("Finished", 0, 0);
                }
            } else {
                emit_route_status_update("CANCELLED", 0, 0);
            }
        });
    })
    .catch((err) => {
        rosnodejs.log.error(err.stack);
    });