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
var path = require('path');

const NavTargets = require('./nav_targets.js');
const TfListener = require('./tf_listener.js');

const nav2_msgs = rclnodejs.require('nav2_msgs').action;
const geometry_msgs = rclnodejs.require('geometry_msgs').msg;
const rosgraph_msgs = rclnodejs.require('rosgraph_msgs').msg;
var clock_message = new rosgraph_msgs.Clock();
const RoutePlanner = require('./route_planner.js');

var multer = require('multer');

const { spawn } = require('child_process');
const { execSync } = require('child_process');

var workingDirectory = process.cwd();
var configDirectory = workingDirectory + '/config';
var mapsDirectory = workingDirectory + '/user_maps'
var configFileName = configDirectory + '/config.json';

var slam_toolbox_install_path;
try {
    slam_toolbox_install_path = execSync('ros2 pkg prefix slam_toolbox').toString();
    slam_toolbox_install_path = slam_toolbox_install_path.substring(0, slam_toolbox_install_path.length - 1);
} catch (error) {
    console.error(`Can not get slam_toolbox path. Is it installed?`);
    console.error(`${error}`);
    return
}

var rap_install_path;
try {
    rap_install_path = execSync('ros2 pkg prefix route_admin_panel').toString();
    rap_install_path = rap_install_path.substring(0, rap_install_path.length - 1);
} catch (error) {
    console.error(`Can not get slam_toolbox path. Is it installed?`);
    console.error(`${error}`);
    return
}

var map_server_install_path;
try {
    map_server_install_path = execSync('ros2 pkg prefix nav2_map_server').toString();
    map_server_install_path = map_server_install_path.substring(0, map_server_install_path.length - 1);
} catch (error) {
    console.error(`Can not get map_server path. Is it installed?`);
    console.error(`${error}`);
    return
}

var amcl_install_path;
try {
    amcl_install_path = execSync('ros2 pkg prefix nav2_amcl').toString();
    amcl_install_path = amcl_install_path.substring(0, amcl_install_path.length - 1);
} catch (error) {
    console.error(`Can not get amcl path. Is it installed?`);
    console.error(`${error}`);
    return
}

var storage = multer.diskStorage({
    destination: function (req, file, cb) {
        cb(null, mapsDirectory);
    },
    filename: function (req, file, cb) {
        cb(null, file.originalname);
    }
});

var upload = multer({ storage: storage });

var initialpose_publisher;
var initial_pose_msg = new geometry_msgs.PoseWithCovarianceStamped();
var robot_initial_pose = {
    robot_pos_x: 0,
    robot_pos_y: 0,
    robot_pos_theta: 0
}

var targets = new NavTargets.TargetList();

var map_data_blob;
var map_metadata;

var map_server_process;
var localization_process;
var localization_flags = {
    enable: false,
    loc_active: false,
    map_active: false,
    restart: false,
    finalize: false
};
var slam_process;
var slam_process_flag;
var slam_process_exited = true;
var autosave_enabled;
var autosave_active = false;
var subprocess_interval;
var selected_map_file = {
    name: '',
    extension: ''
};
var selected_map_mode;
var map_file_name;
var serializePoseGraphClient;
var changeMapServerStateClient;
var changeAMCLStateClient;

var rosNode;

var tfTree = new TfListener.TfTree();
var robot_pose_emit_timestamp;

var nav2_actionClient;

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
    .option('sim_time', {
        alias: 's',
        default: false,
        description: 'Use time provided by simulator',
        type: 'boolean'
    })
    .help()
    .alias('help', 'h')
    .version(false)
    .argv;

console.log("Map scale: [", argv.map_scale_min, ", ", argv.map_scale_max, "]")
console.log("Use simulation time: " + argv.sim_time);

process.on('exit', (code) => {
    console.log(`About to exit with code: ${code}`);
    clearInterval(subprocess_interval);
    slam_process_flag = false;
    localization_flags.enable = false;
    autosave_enabled = false;
    subprocessControl();
});

function save_config(force_restart) {
    let confObject = {
        mapMode: selected_map_mode,
        customMapFileName: selected_map_file.name,
        customMapFileExt: selected_map_file.extension,
        autosaveEnable: autosave_enabled,
        targetList: targets
    }
    console.log("Save config:");
    console.log(confObject);
    let jsonString = JSON.stringify(confObject);
    fs.writeFile(configFileName, jsonString, 'utf8', function (err) {
        if (err) {
            console.log(err);
        }
    });
    load_config(force_restart);
}

function load_config(force_restart) {
    if (fs.existsSync(configFileName)) {
        fs.readFile(configFileName, 'utf8', function (err, data) {
            if (err) {
                console.log(err);
            } else {
                let confObject = JSON.parse(data);
                targets.targets = confObject.targetList.targets;
                if (confObject.mapMode == 'SLAM') {
                    selected_map_mode = confObject.mapMode;
                    slam_process_flag = true;
                    localization_flags.enable = false;
                    autosave_enabled = confObject.autosaveEnable;
                } else if (confObject.mapMode == "STATIC") {
                    selected_map_mode = confObject.mapMode;
                    slam_process_flag = false;
                    autosave_enabled = false;
                    selected_map_file.name = confObject.customMapFileName;
                    selected_map_file.extension = confObject.customMapFileExt;
                    if (force_restart) {
                        localization_flags.restart = true;
                    }
                    localization_flags.enable = true;
                } else {
                    slam_process_flag = false;
                    autosave_enabled = false;
                    localization_flags.enable = false;
                }
            }
        });
    } else {
        console.log("Config file does not exist, create default.");
        if (!fs.existsSync(configDirectory)) {
            fs.mkdirSync(configDirectory);
        }
        selected_map_mode = 'SLAM';
        selected_map_file.name = '';
        selected_map_file.extension = '';
        autosave_enabled = true;
        save_config(false);
    }
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

function setInitialPose(initial_pose) {
    if (selected_map_file.extension == '.yaml') {
        let target_quaternion = math3d.Quaternion.Euler(0, 0, initial_pose.robot_pos_theta * 180 / Math.PI);
        initial_pose_msg.header.frame_id = "map";
        if (argv.sim_time == true) {
            initial_pose_msg.header.stamp.sec = clock_message.clock.sec - 1;
            initial_pose_msg.header.stamp.nanosec = clock_message.clock.nanosec;
        }
        initial_pose_msg.pose.pose.position.x = initial_pose.robot_pos_x;
        initial_pose_msg.pose.pose.position.y = initial_pose.robot_pos_y;
        initial_pose_msg.pose.pose.position.z = 0;
        initial_pose_msg.pose.pose.orientation.x = target_quaternion.x;
        initial_pose_msg.pose.pose.orientation.y = target_quaternion.y;
        initial_pose_msg.pose.pose.orientation.z = target_quaternion.z;
        initial_pose_msg.pose.pose.orientation.w = target_quaternion.w;
        initialpose_publisher.publish(initial_pose_msg);
    } else if (selected_map_file.extension == '.posegraph') {
        robot_initial_pose.robot_pos_x = initial_pose.robot_pos_x;
        robot_initial_pose.robot_pos_y = initial_pose.robot_pos_y;
        robot_initial_pose.robot_pos_theta = initial_pose.robot_pos_theta;
        localization_flags.restart = true;
    }
}

function transition_activate(stateClient) {
    let request = {
        transition: {
            id: 3, // TRANSITION_ACTIVATE
            label: 'activate'
        }
    }
    stateClient.sendRequest(request, response => {
        if (response.success) {
            console.log(stateClient._serviceName + " set to state ACTIVE");
        } else {
            console.log("Can not set " + stateClient._serviceName + " state: ACTIVE");
            // setTimeout(transition_activate, 250, stateClient);
        }
    });
}

function transition_configure(stateClient) {
    let request = {
        transition: {
            id: 1, // TRANSITION_CONFIGURE
            label: 'configure'
        }
    }
    stateClient.sendRequest(request, response => {
        if (response.success) {
            console.log(stateClient._serviceName + " set to state INACTIVE");
            transition_activate(stateClient);
        } else {
            console.log("Can not set " + stateClient._serviceName + " state: INACTIVE");
            // setTimeout(transition_configure, 250, stateClient);
        }
    });
}

function subprocessControl() {
    if (slam_process_flag == true && slam_process_exited == true) {
        startSLAM();
    } else if (slam_process_flag == false && slam_process_exited == false) {
        stopSLAM();
    }

    if (localization_flags.restart) {
        if (localization_flags.loc_active || localization_flags.map_active) {
            stopLocalization();
        } else {
            console.log(`Set localization_flags.restart to [false]`);
            localization_flags.restart = false;
        }
    } else if (localization_flags.finalize == true) {
        console.log("Wait for finalization");
    } else {
        if (localization_flags.enable == true && (localization_flags.loc_active == false || localization_flags.map_active == false)) {
            startLocalization();
        } else if (localization_flags.enable == false && (localization_flags.loc_active == true || localization_flags.map_active == true)) {
            stopLocalization();
        }
    }

    if (autosave_active == true && autosave_enabled == true) {
        saveMap(map_file_name);
    } else if (autosave_active == true && autosave_enabled == false) {
        autosave_active = false;
    } else if (autosave_active == false && autosave_enabled == true) {
        let date_time = new Date();
        let date_string = date_time.toISOString().replace('T', '_').substr(0, 16);
        map_file_name = mapsDirectory + '/auto_saved_map_' + date_string;
        autosave_active = true;
    }
}

function stopLocalization() {
    if (localization_process) {
        console.log("Stop localization process.");
        localization_flags.finalize = true;
        localization_process.kill('SIGTERM');
        localization_process.kill('SIGINT');
        localization_process.kill('SIGKILL');
    }

    if (map_server_process) {
        console.log("Stop map_server process.");
        localization_flags.finalize = true;
        map_server_process.kill('SIGTERM');
        map_server_process.kill('SIGINT');
        map_server_process.kill('SIGKILL');
    }
}

function startLocalization() {
    if (selected_map_file.extension == '.posegraph') {
        if (fs.existsSync(rap_install_path + '/share/route_admin_panel/config/slam_toolbox.yaml')) {
            fs.readFile(rap_install_path + '/share/route_admin_panel/config/slam_toolbox.yaml', 'utf8', function (err, data) {
                if (err) {
                    console.log(err);
                } else {
                    if (argv.sim_time) {
                        data += '    use_sim_time: true\n';
                    } else {
                        data += '    use_sim_time: false\n';
                    }
                    data += '    map_file_name: ' + mapsDirectory + '/' + selected_map_file.name + '\n';
                    data += '    map_start_pose: ['
                        + robot_initial_pose.robot_pos_x.toFixed(3) + ', '
                        + robot_initial_pose.robot_pos_y.toFixed(3) + ', '
                        + robot_initial_pose.robot_pos_theta.toFixed(4) + ']\n';
                    data += '    mode: localization\n';
                    fs.writeFile(configDirectory + '/params.yaml', data, function (err) {
                        if (err) {
                            console.log('Could not write slam toolbox configuration file');
                            return console.log(err);
                        }
                    });
                }
            });
        } else {
            console.log("Slam toolbox config file not found, can not start localization.");
            return
        }
        localization_process = spawn(slam_toolbox_install_path + '/lib/slam_toolbox/localization_slam_toolbox_node', ['__params:=' + configDirectory + '/params.yaml']);
        // localization_process.stdout.on('data', (data) => { console.log(`stdout: ${data}`); });
        // localization_process.stderr.on('data', (data) => { console.error(`stderr: ${data}`); });
        localization_process.on('close', (code) => {
            localization_flags.loc_active = false;
            localization_flags.finalize = false;
        });
        localization_flags.loc_active = true;
    } else if (selected_map_file.extension == '.yaml') {
        if (localization_flags.map_active == false) {
            let map_server_params = 'map_server:\n';
            map_server_params += '  ros__parameters:\n';
            map_server_params += '    yaml_filename: ' + mapsDirectory + '/' + selected_map_file.name + selected_map_file.extension + '\n';
            if (argv.sim_time) {
                map_server_params += '    use_sim_time: true\n';
            } else {
                map_server_params += '    use_sim_time: false\n';
            }
            fs.writeFile(configDirectory + '/map_server_params.yaml', map_server_params, function (err) {
                if (err) {
                    console.log('Could not write map server configuration file');
                    return console.log(err);
                }
            });
            map_server_process = spawn(map_server_install_path + '/lib/nav2_map_server/map_server', ['__params:=' + configDirectory + '/map_server_params.yaml']);
            map_server_process.stdout.on('data', (data) => { console.log(`[ map_server_process ] - stdout: ${data}`); });
            map_server_process.stderr.on('data', (data) => { console.error(`[ map_server_process ] - stderr: ${data}`); });
            map_server_process.on('close', (code) => {
                localization_flags.map_active = false;
                if (localization_flags.map_active == false && localization_flags.loc_active == false) {
                    localization_flags.finalize = false;
                }
            });

            changeMapServerStateClient.waitForService(1000).then(result => {
                console.log("/map_server/change_state wait done");
                if (!result) {
                    console.log('Error: /map_server/change_state: not available');
                    return;
                }
                setTimeout(transition_configure, 500, changeMapServerStateClient);
                localization_flags.map_active = true;
            });
        }

        if (localization_flags.loc_active == false) {
            let amcl_params = 'amcl:\n';
            amcl_params += '  ros__parameters:\n';
            amcl_params += '    odom_frame_id: odom\n';
            amcl_params += '    odom_model_type: diff-corrected\n';
            amcl_params += '    base_frame_id: base_link\n';
            amcl_params += '    update_min_d: 0.1\n';
            amcl_params += '    update_min_a: 0.2\n';
            amcl_params += '    min_particles: 500\n';
            if (argv.sim_time) {
                amcl_params += '    use_sim_time: true\n';
            } else {
                amcl_params += '    use_sim_time: false\n';
            }
            fs.writeFile(configDirectory + '/params.yaml', amcl_params, function (err) {
                if (err) {
                    console.log('Could not write amcl configuration file');
                    return console.log(err);
                }
            });
            localization_process = spawn(amcl_install_path + '/lib/nav2_amcl/amcl', ['__params:=' + configDirectory + '/params.yaml']);
            changeAMCLStateClient.waitForService(1000).then(result => {
                if (!result) {
                    console.log('Error: /amcl/change_state: not available');
                    return;
                }
                setTimeout(transition_configure, 500, changeAMCLStateClient);
                robot_initial_pose.robot_pos_x = 0;
                robot_initial_pose.robot_pos_y = 0;
                robot_initial_pose.robot_pos_theta = 0;
                setTimeout(setInitialPose, 1500, robot_initial_pose);
            });
        }
        localization_process.stdout.on('data', (data) => { console.log(`[ AMCL ] stdout: ${data}`); });
        localization_process.stderr.on('data', (data) => { console.error(`[ AMCL ] stderr: ${data}`); });
        localization_process.on('close', (code) => {
            localization_flags.loc_active = false;
            if (localization_flags.map_active == false && localization_flags.loc_active == false) {
                localization_flags.finalize = false;
            }
        });
        localization_flags.loc_active = true;
    } else {
        console.log('Unsupported map file type');
        return;
    }
}

function stopSLAM() {
    console.log("Stop slam process");
    if (slam_process) {
        console.log("Slam process found");
        slam_process.kill('SIGTERM');
        slam_process.kill('SIGINT');
        slam_process.kill('SIGKILL');
    }
}

function startSLAM() {
    if (fs.existsSync(rap_install_path + '/share/route_admin_panel/config/slam_toolbox.yaml')) {
        fs.readFile(rap_install_path + '/share/route_admin_panel/config/slam_toolbox.yaml', 'utf8', function (err, data) {
            if (err) {
                console.log(err);
            } else {
                if (argv.sim_time) {
                    data += '    use_sim_time: true\n';
                } else {
                    data += '    use_sim_time: false\n';
                }
                data += '    mode: mapping\n';
                fs.writeFile(configDirectory + '/params.yaml', data, function (err) {
                    if (err) {
                        console.log('Could not write slam toolbox configuration file')
                        return console.log(err);
                    }
                });
            }
        });
    } else {
        console.log("Slam toolbox config file not found, can not start mapping.");
        return
    }
    slam_process = spawn(slam_toolbox_install_path + '/lib/slam_toolbox/sync_slam_toolbox_node', ['__params:=' + configDirectory + '/params.yaml']);
    slam_process.stdout.on('data', (data) => {console.log(`stdout: ${data}`);});
    slam_process.stderr.on('data', (data) => {console.error(`stderr: ${data}`);});
    slam_process.on('close', (code) => {
        console.log(`child process exited with code ${code}`);
        slam_process_exited = true;
    });
    console.log("Slam toolbox launched");
    slam_process_exited = false;
}

function saveMap(filename) {
    console.log(`Saving map with name: ${filename}`);

    let request = {
        filename: filename
    };

    serializePoseGraphClient.waitForService(1000).then(result => {
        if (!result) {
            console.log('Error: /slam_toolbox/serialize_map not available');
            return;
        }
        serializePoseGraphClient.sendRequest(request, response => {
            console.log("Map saved");
            update_map_filenames();
        });
    });
}

app.get('/', function (req, res) {
    res.sendFile(__dirname + '/public/index.html');
});

app.use(express.static('public'))

app.post('/upload', upload.single('map-image'), function (req, res) {
    console.log("save yaml file fo rmap");
    let uploaded_map_file = mapsDirectory + '/' + req.file.originalname + '.yaml';
    let imagePath = req.file.path.replace(/^user_maps\//, '');
    let yaml_ouptut = 'image: ' + imagePath + '\n';
    yaml_ouptut += 'resolution: ' + req.body.mapResolution + '\n';
    yaml_ouptut += 'origin: [0.0, 0.0, 0.0]\n';
    yaml_ouptut += 'occupied_thresh: 0.65\n';
    yaml_ouptut += 'free_thresh: 0.196\n';
    yaml_ouptut += 'negate: 0\n';
    console.log('yaml_output:');
    console.log(yaml_ouptut);
    console.log('uploaded_map_file');
    console.log(uploaded_map_file);
    fs.writeFile(uploaded_map_file, yaml_ouptut, function (err) {
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
    selected_map_file.name = settings.map_file_name;
    selected_map_file.extension = settings.map_file_extension;
    autosave_enabled = settings.autosave_enabled;
    save_config(true);
}

function update_map_filenames() {
    let posegraph_filenames = getFileList(mapsDirectory, '.posegraph');
    let yaml_filenames = getFileList(mapsDirectory, '.yaml');
    let filenames = posegraph_filenames.concat(yaml_filenames);
    io.emit('map_file_list', filenames);
}

function getFileList(startPath, extension) {
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
            getFileList(filename, extension); //recurse
        }
        else if (filename.indexOf(extension) >= 0) {
            file_names.push({
                name: files[i].substring(0, files[i].length - extension.length),
                extension: extension
            });
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
    if (argv.sim_time == true) {
        action_goal._refObject.pose.header.stamp.sec = clock_message.clock.sec;
    } else {

    }
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

    socket.on('delete_target', function (target_id) {
        targets.remove_target(target_id);
        emit_target_delete(target_id);
        save_config(false);
    });

    socket.on('new_target', function (new_target) {
        console.log("New target\n", new_target);
        let target = new NavTargets.Target(targets.get_next_id(), new_target.x, new_target.y, new_target.theta, new_target.label);
        targets.add_target(target);
        emit_target(target);
        save_config(false);
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

http.listen(8000, function () {
    if (!fs.existsSync(configDirectory)) {
        fs.mkdirSync(configDirectory);
    }
    if (!fs.existsSync(mapsDirectory)) {
        fs.mkdirSync(mapsDirectory);
    }
    console.log('listening on *:8000');
});

rclnodejs.init().then(() => {
    rosNode = rclnodejs.createNode('rap_server_node');
    robot_pose_emit_timestamp = Date.now();

    if (argv.sim_time == true) {
        rosNode.createSubscription('rosgraph_msgs/msg/Clock', '/clock',
            (data) => {
                clock_message = data;
            }, {
            queueSize: 1,
            throttleMs: 0
        });
    }

    initialpose_publisher = rosNode.createPublisher('geometry_msgs/msg/PoseWithCovarianceStamped', '/initialpose');
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

    serializePoseGraphClient = rosNode.createClient('slam_toolbox/srvs/SerializePoseGraph', '/serialize_map');

    changeMapServerStateClient = rosNode.createClient('lifecycle_msgs/srv/ChangeState', '/map_server/change_state');
    changeAMCLStateClient = rosNode.createClient('lifecycle_msgs/srv/ChangeState', '/amcl/change_state');

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

load_config(false);
subprocess_interval = setInterval(subprocessControl, 1000);