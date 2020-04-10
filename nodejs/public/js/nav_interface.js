var socket;
var targets_table;
var targets_array = [];
var route_array = [];

function addTableRow(table, target_params) {
    //  Check if entry already exists
    if (document.getElementById('targetRow#' + target_params.id)) {
        return
    }
    let row = table.insertRow(table.rows.length);
    row.id = 'targetRow#' + target_params.id;
    row.insertCell(-1).innerHTML = target_params.x.toFixed(2);
    row.insertCell(-1).innerHTML = target_params.y.toFixed(2);
    row.insertCell(-1).innerHTML = (target_params.theta * 180 / Math.PI).toFixed(2);
    row.insertCell(-1).innerHTML = target_params.label;
    let buttonsCell = row.insertCell(-1);
    buttonsCell.appendChild(createDriveButtonElement(target_params.id));
    buttonsCell.appendChild(createDeleteButtonElement(target_params.id));
}

var stage;
var arrowsLayer;
var robot_arrow;
var robot_position = { x: 0, y: 0, theta: 0 };
var currentRobotPosition = { x: 0, y: 0, theta: 0 }
var newTargetLayer;
var newTargetArrow;

var mapLayer;
var konva_map_image;
var map_metadata;
var natural_map_size;
var map_image_element;
var map_scale;
var saveCurrentPosDialog;
var mapSettingsDialog;
var mapSettingsTab;
var mapUploadTab;
var mapFileDropdown;
var mapModeRadioStatic;
var mapModeRadioSLAM;
var mapAutoSaveCheckbox;
var uploadMapProgressBar;
var pathCreatorDialog
var routeManagerDialog;
var routeStatusLabel;
var loading_dropdown;
var unloading_dropdown;
var currentPath;
var currentPosesArray;

function setTargetLabelsCheckboxListener() {
    let checkbox = document.getElementById('targetLabelsCheckbox');
    let labelsContainer = document.getElementById('labels-container');
    checkbox.addEventListener('change', (event) => {
        if (event.target.checked) {
            labelsContainer.style.display = 'inherit';
        } else {
            labelsContainer.style.display = 'none';
        }
    });
    checkbox.checked = true;
}

function setTargetsTableCheckboxListener() {
    let checkbox = document.getElementById('targetsTableCheckbox');
    let tableContainer = document.getElementById('table-container');
    checkbox.addEventListener('change', (event) => {
        if (event.target.checked) {
            tableContainer.style.display = 'inherit';
        } else {
            tableContainer.style.display = 'none';
        }
    });
    checkbox.checked = true;
}

function setZoomSliderCheckboxListener() {
    let checkbox = document.getElementById('zoomCheckbox');
    let zoomContainer = document.getElementById('zoom-container');
    checkbox.addEventListener('change', (event) => {
        if (event.target.checked) {
            zoomContainer.style.display = 'inherit';
        } else {
            zoomContainer.style.display = 'none';
        }
    });
    checkbox.checked = true;
}

function cancelNewTarget() {
    document.getElementById('konva-container').removeEventListener('mousedown', initNewTargetArrow);
    document.getElementById('konva-container').removeEventListener('mousemove', rotateNewTargetArrow);
    document.getElementById('konva-container').removeEventListener('mousedown', acceptNewTarget);
    if (newTargetArrow) {
        newTargetArrow.remove();
    }
    if (newTargetLayer) {
        newTargetLayer.remove();
    }
    stage.draw();
}

function acceptNewTarget(event) {
    document.getElementById('konva-container').removeEventListener('mousemove', rotateNewTargetArrow);
    document.getElementById('konva-container').removeEventListener('mousedown', acceptNewTarget);
    let arrowTipX = event.clientX - newTargetArrow.attrs.x;
    let arrowTipY = newTargetArrow.attrs.y - event.clientY;
    let arrowAngle = Math.atan2(arrowTipY, arrowTipX);
    newTargetArrow.remove();
    newTargetLayer.remove();
    stage.draw();
    let targetX = (newTargetArrow.attrs.x - stage.width() / 2) * map_metadata.resolution / map_scale + robot_position.x;
    let targetY = (-newTargetArrow.attrs.y + stage.height() / 2) * map_metadata.resolution / map_scale + robot_position.y;
    saveCurrentPosition(targetX, targetY, arrowAngle);
}

function rotateNewTargetArrow(event) {
    let arrowTipX = event.clientX - newTargetArrow.attrs.x;
    let arrowTipY = event.clientY - newTargetArrow.attrs.y;
    let arrowBaseX = 0;
    let arrowBaseY = 0;
    newTargetArrow.attrs.points = [arrowBaseX, arrowBaseY, arrowTipX, arrowTipY];
    stage.draw();
}

function initNewTargetArrow(event) {
    newTargetArrow = new Konva.Arrow({
        x: event.clientX,
        y: event.clientY,
        points: [0, 0, 0, 0],
        pointerLength: 20,
        pointerWidth: 10,
        fill: 'green',
        stroke: 'green',
        strokeWidth: 3
    });
    newTargetLayer.add(newTargetArrow);
    document.getElementById('konva-container').removeEventListener('mousedown', initNewTargetArrow);
    document.getElementById('konva-container').addEventListener('mousemove', rotateNewTargetArrow);
    document.getElementById('konva-container').addEventListener('mousedown', acceptNewTarget);
}

$(document).keydown(function (e) {
    if (e.key === "Escape") { // escape key maps to keycode `27`
        console.log("Escape pressed");
        dismissAllDialogs();
    }
});

function dismissAllDialogs() {
    dismissCurrentPosDialog();
    dismissPathCreatorDialog();
    dismissRouteManagerDialog();
    dismissMapSettingsDialog();
    cancelNewTarget();
    cancelRobotPose();
}



function addTargetDialog() {
    newTargetLayer = new Konva.Layer();
    stage.add(newTargetLayer);
    document.getElementById('konva-container').addEventListener('mousedown', initNewTargetArrow);
}

function showMapSettingsDialog() {
    dismissAllDialogs();
    showMapSettingsTab();
    mapSettingsDialog.style.display = "block";
    if (mapModeRadioStatic.checked) {
        enableMapFileDropdown();
    } else {
        disableMapFileDropdown();
    }
}

function saveMapSettings() {
    dismissMapSettingsDialog();
    let map_settings = {
        map_static: mapModeRadioStatic.checked,
        map_slam: mapModeRadioSLAM.checked,
        map_file: mapFileDropdown.value,
        map_autosave: mapAutoSaveCheckbox.checked
    };
    console.log(map_settings);
    socket.emit('save_map_settings', map_settings);
}

function showMapSettingsTab() {
    mapSettingsTab.style.display = "block";
    mapUploadTab.style.display = "none";
}

function showMapUploadTab() {
    mapSettingsTab.style.display = "none";
    mapUploadTab.style.display = "block";
}

function enableMapFileDropdown() {
    mapFileDropdown.disabled = false;
    disableAutoSaveCheckbox();
}

function disableMapFileDropdown() {
    mapFileDropdown.disabled = true;
    enableAutoSaveCheckbox();
}

function enableAutoSaveCheckbox() {
    mapAutoSaveCheckbox.disabled = false;
}

function disableAutoSaveCheckbox() {
    mapAutoSaveCheckbox.disabled = true;
}

function updateMapFilenames(filenames) {
    while (mapFileDropdown.length > 0) {
        mapFileDropdown.removeChild(mapFileDropdown[0]);
    }
    filenames.forEach(name => add_map_option(name));
}

function add_map_option(filename) {
    let option = document.createElement("OPTION");
    option.value = filename;
    option.text = filename;
    mapFileDropdown.add(option);
}

function showRouteManagerDialog() {
    dismissAllDialogs();
    let existing_poses_table = document.getElementById('existing_poses').getElementsByTagName('tbody')[0];
    for (var i = existing_poses_table.rows.length - 1; i >= 0; i--) {
        existing_poses_table.deleteRow(i);
    }
    targets_array.forEach(target => {
        let row = existing_poses_table.insertRow(existing_poses_table.rows.length);
        row.id = 'poseRow#' + target.id;
        row.insertCell(-1).innerHTML = "[" + target.x.toFixed(2) + ", " + target.y.toFixed(2) + ", " + (target.theta * 180 / Math.PI).toFixed(2) + "]";
        row.insertCell(-1).innerHTML = target.label;
        let buttonsCell = row.insertCell(-1);
        buttonsCell.appendChild(createAddToRouteButtonElement(target.id));
        buttonsCell.classList.add("text-center");
    });

    routeManagerDialog.style.display = "block";
}

function showPathCreatorDialog() {
    dismissAllDialogs();
    while (loading_dropdown.firstChild) {
        loading_dropdown.removeChild(loading_dropdown.firstChild);
    }

    while (unloading_dropdown.firstChild) {
        unloading_dropdown.removeChild(unloading_dropdown.firstChild);
    }

    targets_array.forEach(target => {
        var opt_l = document.createElement('option');
        var opt_u = document.createElement('option');
        opt_l.appendChild(document.createTextNode(target.label));
        opt_u.appendChild(document.createTextNode(target.label));
        opt_l.value = target.id;
        opt_u.value = target.id;
        loading_dropdown.appendChild(opt_l);
        unloading_dropdown.appendChild(opt_u);
    });
    pathCreatorDialog.style.display = "block";
}

function dismissRouteManagerDialog() {
    routeManagerDialog.style.display = "none";
}

function dismissMapSettingsDialog() {
    mapSettingsDialog.style.display = "none";
    uploadMapProgressBar.style.display = "none";
}

function dismissPathCreatorDialog() {
    pathCreatorDialog.style.display = "none";
}

function createPath() {
    let plan_request = {
        start: loading_dropdown.value,
        end: unloading_dropdown.value
    };
    socket.emit('make_plan', plan_request);
    dismissPathCreatorDialog();
}

function uploadCustomMapConfirm() {
    form = document.getElementById("map-upload-form");
    $.ajax({
        url: 'upload',
        type: 'POST',
        data: new FormData(form),
        cache: false,
        contentType: false,
        processData: false,
        xhr: function () {
            var myXhr = $.ajaxSettings.xhr();
            if (myXhr.upload) {
                uploadMapProgressBar.style.display = "block";
                myXhr.upload.addEventListener('progress', function (e) {
                    if (e.lengthComputable) {
                        $('progress').attr({
                            value: e.loaded,
                            max: e.total,
                        });
                    }
                }, false);
                myXhr.upload.addEventListener('loadend', function (e) {
                    uploadMapProgressBar.style.display = "none";
                }, false);
            }
            return myXhr;
        }
    });
}

function setRobotPose() {
    dismissAllDialogs();
    newTargetLayer = new Konva.Layer();
    stage.add(newTargetLayer);
    document.getElementById('konva-container').addEventListener('mousedown', initRobotPoseArrow);
}


function cancelRobotPose() {
    document.getElementById('konva-container').removeEventListener('mousedown', initRobotPoseArrow);
    document.getElementById('konva-container').removeEventListener('mousemove', rotateRobotPoseArrow);
    document.getElementById('konva-container').removeEventListener('mousedown', acceptRobotPose);
    if (newTargetArrow) {
        newTargetArrow.remove();
    }
    if (newTargetLayer) {
        newTargetLayer.remove();
    }
    stage.draw();
}

function acceptRobotPose(event) {
    document.getElementById('konva-container').removeEventListener('mousemove', rotateRobotPoseArrow);
    document.getElementById('konva-container').removeEventListener('mousedown', acceptRobotPose);
    let arrowTipX = event.clientX - newTargetArrow.attrs.x;
    let arrowTipY = newTargetArrow.attrs.y - event.clientY;
    let arrowAngle = Math.atan2(arrowTipY, arrowTipX);
    newTargetArrow.remove();
    newTargetLayer.remove();
    stage.draw();
    let targetX = (newTargetArrow.attrs.x - stage.width() / 2) * map_metadata.resolution / map_scale + robot_position.x;
    let targetY = (-newTargetArrow.attrs.y + stage.height() / 2) * map_metadata.resolution / map_scale + robot_position.y;
    updateRobotPose(targetX, targetY, arrowAngle);
}

function rotateRobotPoseArrow(event) {
    let arrowTipX = event.clientX - newTargetArrow.attrs.x;
    let arrowTipY = event.clientY - newTargetArrow.attrs.y;
    let arrowBaseX = 0;
    let arrowBaseY = 0;
    newTargetArrow.attrs.points = [arrowBaseX, arrowBaseY, arrowTipX, arrowTipY];
    stage.draw();
}

function initRobotPoseArrow(event) {
    newTargetArrow = new Konva.Arrow({
        x: event.clientX,
        y: event.clientY,
        points: [0, 0, 0, 0],
        pointerLength: 20,
        pointerWidth: 10,
        fill: 'green',
        stroke: 'green',
        strokeWidth: 3
    });
    newTargetLayer.add(newTargetArrow);
    document.getElementById('konva-container').removeEventListener('mousedown', initRobotPoseArrow);
    document.getElementById('konva-container').addEventListener('mousemove', rotateRobotPoseArrow);
    document.getElementById('konva-container').addEventListener('mousedown', acceptRobotPose);
}

function updateRobotPose(x, y, theta) {
    dismissAllDialogs();
    if (x) {
        currentRobotPosition.x = x;
    } else {
        currentRobotPosition.x = robot_position.x;
    }
    if (y) {
        currentRobotPosition.y = y;
    } else {
        currentRobotPosition.y = robot_position.y;
    }
    if (theta) {
        currentRobotPosition.theta = theta;
    } else {
        currentRobotPosition.theta = robot_position.theta;
    }
    let robot_pose = {
        robot_pos_x: currentRobotPosition.x,
        robot_pos_y: currentRobotPosition.y,
        robot_pos_theta: currentRobotPosition.theta
    }
    socket.emit('set_initialpose', robot_pose);
}

function saveCurrentPosition(x, y, theta) {
    dismissAllDialogs();
    let currentX_cell = document.getElementById('currentPosX');
    let currentY_cell = document.getElementById('currentPosY');
    let currentTheta_cell = document.getElementById('currentPosTheta');
    if (x) {
        currentRobotPosition.x = x;
    } else {
        currentRobotPosition.x = robot_position.x;
    }
    if (y) {
        currentRobotPosition.y = y;
    } else {
        currentRobotPosition.y = robot_position.y;
    }
    if (theta) {
        currentRobotPosition.theta = theta;
    } else {
        currentRobotPosition.theta = robot_position.theta;
    }
    currentX_cell.innerHTML = currentRobotPosition.x.toFixed(2);
    currentY_cell.innerHTML = currentRobotPosition.y.toFixed(2);
    currentTheta_cell.innerHTML = (currentRobotPosition.theta * 180 / Math.PI).toFixed(2);
    saveCurrentPosDialog.style.display = "block";
}

window.onclick = function (event) {
    if (event.target == saveCurrentPosDialog) {
        dismissAllDialogs();
    } else if (event.target == mapSettingsDialog) {
        dismissAllDialogs();
    } else if (event.target == routeManagerDialog) {
        dismissAllDialogs();
    } else if (event.target == pathCreatorDialog) {
        dismissAllDialogs();
    }
}

function dismissCurrentPosDialog() {
    saveCurrentPosDialog.style.display = "none";
}

function saveCurrentPosConfirm() {
    let position_label = document.getElementById('currentPosLabel').value;
    socket.emit('new_target', {
        label: position_label,
        x: currentRobotPosition.x,
        y: currentRobotPosition.y,
        theta: currentRobotPosition.theta
    });
    dismissCurrentPosDialog();
}

function initRobotArrow() {
    robot_arrow = new Konva.Arrow({
        x: stage.width() / 2,
        y: stage.height() / 2,
        points: [0, 0, 0, 0],
        pointerLength: 20,
        pointerWidth: 10,
        fill: 'green',
        stroke: 'red',
        strokeWidth: 3
    });
    arrowsLayer.add(robot_arrow);
}

function updateRobotArrow(theta) {
    robot_arrow.attrs.points = [-25 * Math.cos(theta), 25 * Math.sin(theta), 25 * Math.cos(theta), -25 * Math.sin(theta)];
    stage.draw();
}

function redrawTargetArrow(arrow_id, arrow_x_center, arrow_y_center, theta) {
    arrowOptions = {
        x: arrow_x_center,
        y: arrow_y_center,
        points: [-25 * Math.cos(theta), 25 * Math.sin(theta), 25 * Math.cos(theta), -25 * Math.sin(theta)],
        pointerLength: 20,
        pointerWidth: 10,
        fill: 'green',
        stroke: 'blue',
        strokeWidth: 3,
        id: 'targetArrow#' + arrow_id
    }
    let tmp_arrow = stage.find("#targetArrow#" + arrow_id)[0];
    if (tmp_arrow) {
        tmp_arrow.setAttrs(arrowOptions);
    } else {
        let targetArrow = new Konva.Arrow(arrowOptions);
        arrowsLayer.add(targetArrow);
    }
}

function updateTarget(target) {
    if (map_metadata) {
        let target_x_screen =
            (stage.width() / 2)
            - map_scale * robot_position.x / map_metadata.resolution
            + map_scale * parseFloat(target.x) / map_metadata.resolution;
        let target_y_screen =
            (stage.height() / 2)
            + map_scale * robot_position.y / map_metadata.resolution
            - map_scale * parseFloat(target.y) / map_metadata.resolution;
        redrawTargetArrow(target.id, target_x_screen, target_y_screen, target.theta);
        updateDestiantionLabel(target.id, target_x_screen, target_y_screen, target.theta, target.label);
    }
}

function updateTargets() {
    targets_array.forEach(updateTarget);
    arrowsLayer.batchDraw();
}

function redrawPath(posesArray) {
    if (posesArray) {
        currentPosesArray = posesArray;
    }
    if (currentPosesArray.length != 0) {
        let line_points = [];
        for (var i = 0; i < currentPosesArray.length; i++) {
            let target_x_screen = (stage.width() / 2) - map_scale * robot_position.x / map_metadata.resolution + map_scale * currentPosesArray[i].pose.position.x / map_metadata.resolution;
            let target_y_screen = (stage.height() / 2) + map_scale * robot_position.y / map_metadata.resolution - map_scale * currentPosesArray[i].pose.position.y / map_metadata.resolution;
            line_points.push(target_x_screen);
            line_points.push(target_y_screen);
        }
        currentPath.setAttrs({
            x: 0,
            y: 0,
            points: line_points,
            stroke: 'red',
            tension: 0
        })
        arrowsLayer.batchDraw();
    }
}


function driveToTarget(target_id) {
    console.log("Send drive to target", target_id, "signal");
    socket.emit('drive_to_target', target_id);
}

function startRoute() {
    let mode = document.querySelector('input[name="mode"]:checked').value;
    socket.emit('route_start', mode);
    dismissRouteManagerDialog();
}

function stopRoute() {
    socket.emit('route_stop');
}

function addToRoute(target_id) {
    console.log("Add " + target_id + " to route");
    socket.emit('add_to_route', target_id);
}

function removeFromRoute(target_id) {
    console.log("Remove " + target_id + " from route");
    socket.emit('remove_from_route', target_id);
}

function moveUpOnRoute(target_id) {
    console.log("Move " + target_id + " up on route");
    socket.emit('move_up_on_route', target_id);
}

function moveDownOnRoute(target_id) {
    console.log("Move " + target_id + " down on route");
    socket.emit('move_down_on_route', target_id);
}

function addRouteTableRow(target, sequence, routeID) {
    let route_table = document.getElementById('route_table').getElementsByTagName('tbody')[0];
    if (sequence > route_table.rows.length) {
        sequence = route_table.rows.length;
    }
    let row = route_table.insertRow(sequence);
    row.id = 'routeRow#' + routeID;
    row.insertCell(-1).innerHTML = "[" + target.x.toFixed(2) + ", " + target.y.toFixed(2) + ", " + (target.theta * 180 / Math.PI).toFixed(2) + "]";
    row.insertCell(-1).innerHTML = target.label;
    let buttonsCell = row.insertCell(-1);
    buttonsCell.appendChild(createMoveUpOnRouteButtonElement(routeID));
    buttonsCell.appendChild(createMoveDownOnRouteButtonElement(routeID));
    buttonsCell.appendChild(createRemoveFromRouteButtonElement(routeID));
    buttonsCell.classList.add("text-center");
}

function removeRouteTableRow(routeID) {
    let tableRow = document.getElementById('routeRow#' + routeID);
    if (tableRow) {
        tableRow.parentNode.removeChild(tableRow);
    }
}

function deleteTarget(target_id) {
    socket.emit('delete_target', target_id);
}

function createIcon(fa_icon_name) {
    let customIcon = document.createElement("i");
    customIcon.classList.add("fa");
    customIcon.classList.add(fa_icon_name);
    return customIcon;
}

function createButton() {
    let customButton = document.createElement("button");
    customButton.classList.add("btn");
    customButton.classList.add("btn-danger");
    customButton.classList.add("btn-sm");
    customButton.classList.add("mx-1");
    customButton.classList.add("my-0");
    return customButton;
}

function createAddToRouteButtonElement(target_id) {
    let addToPathButton = createButton();
    addToPathButton.onclick = function () { addToRoute(target_id); };
    addToPathButton.appendChild(createIcon('fa-plus-square'));
    return addToPathButton;
}

function createRemoveFromRouteButtonElement(target_id) {
    let removeFromPathButton = createButton();
    removeFromPathButton.onclick = function () { removeFromRoute(target_id); };
    removeFromPathButton.appendChild(createIcon('fa-trash-alt'));
    return removeFromPathButton;
}

function createMoveUpOnRouteButtonElement(target_id) {
    let moveUpButton = createButton();
    moveUpButton.onclick = function () { moveUpOnRoute(target_id); };
    moveUpButton.appendChild(createIcon('fa-arrow-up'));
    return moveUpButton;
}

function createMoveDownOnRouteButtonElement(target_id) {
    let moveDownButton = createButton();
    moveDownButton.onclick = function () { moveDownOnRoute(target_id); };
    moveDownButton.appendChild(createIcon('fa-arrow-down'));
    return moveDownButton;
}

function createDriveButtonElement(target_id) {
    let driveButton = createButton();
    driveButton.onclick = function () { driveToTarget(target_id); };
    driveButton.appendChild(createIcon('fa-angle-double-right'));
    return driveButton;
}

function createDeleteButtonElement(target_id) {
    let deleteButton = createButton();
    deleteButton.onclick = function () { deleteTarget(target_id); };
    deleteButton.appendChild(createIcon('fa-trash-alt'));
    return deleteButton;
}

function isLabelWithinScreen(label_x_pos, label_y_pos) {
    if (label_x_pos + 200 < window.innerWidth && label_x_pos > 0) {
        if (label_y_pos + 241 < window.innerHeight && label_y_pos > 0) {
            return true;
        }
    }
    return false;
}

function updateDestiantionLabel(target_id, pos_x, pos_y, pos_theta, label_text) {
    let labelDiv = document.getElementById('label#' + target_id);

    if (labelDiv) {
        if (isLabelWithinScreen(pos_x, pos_y)) {
            labelDiv.style.left = pos_x + "px";
            labelDiv.style.top = pos_y + "px";
        } else {
            labelDiv.parentNode.removeChild(labelDiv);
        }
    } else {
        if (isLabelWithinScreen(pos_x, pos_y)) {
            let labelDiv = document.createElement("div");
            labelDiv.id = 'label#' + target_id;
            labelDiv.classList.add("alert");
            labelDiv.classList.add("alert-primary");
            labelDiv.classList.add("dest-on-map");
            labelDiv.classList.add("p-1");
            labelDiv.appendChild(document.createTextNode(label_text));
            labelDiv.appendChild(createDriveButtonElement(target_id));
            labelDiv.appendChild(createDeleteButtonElement(target_id));
            labelDiv.style.left = pos_x + "px";
            labelDiv.style.top = pos_y + "px";
            document.getElementById("labels-container").appendChild(labelDiv);
        }
    }
}

function redraw_map(scale, robot_x_pos, robot_y_pos) {
    if (map_image_element) {
        mapLayer.removeChildren();
        map_image_element.setAttrs({
            x:
                (stage.width() / 2)
                + scale * map_metadata.origin.position.x / map_metadata.resolution
                - scale * robot_x_pos / map_metadata.resolution,
            y:
                (stage.height() / 2)
                - scale * map_metadata.height
                - scale * map_metadata.origin.position.y / map_metadata.resolution
                + scale * robot_y_pos / map_metadata.resolution,
            scaleX: scale,
            scaleY: scale
        });
        mapLayer.add(map_image_element);
        map_image_element.moveToBottom();
        mapLayer.batchDraw();
        updateTargets();
        redrawPath();
    }
}

function mapResizeCallback(event) {
    if (event.additionalEvent == 'pinchin') {
        mapZoomSlider.value = parseInt(mapZoomSlider.value) - 1;
        map_scale = parseInt(mapZoomSlider.value, 10) / 100;
        redraw_map(map_scale, robot_position.x, robot_position.y);
    } else if (event.additionalEvent == 'pinchout') {
        mapZoomSlider.value = parseInt(mapZoomSlider.value) + 1;
        map_scale = parseInt(mapZoomSlider.value, 10) / 100;
        redraw_map(map_scale, robot_position.x, robot_position.y);
    } else if (event.deltaY < 0) {
        mapZoomSlider.value = parseInt(mapZoomSlider.value) + 1;
    } else if (event.deltaY > 0) {
        mapZoomSlider.value = parseInt(mapZoomSlider.value) - 1;
    }
    map_scale = parseInt(mapZoomSlider.value, 10) / 100;
    redraw_map(map_scale, robot_position.x, robot_position.y);
}

window.onload = function () {
    socket = io();

    let stageDiv = document.getElementById('konva-container');
    stage = new Konva.Stage({
        container: 'konva-container', // id of container <div>
        width: stageDiv.offsetWidth,
        height: stageDiv.offsetHeight
    });
    arrowsLayer = new Konva.Layer();
    mapLayer = new Konva.Layer();

    initRobotArrow();
    stage.add(mapLayer);
    stage.add(arrowsLayer);
    mapLayer.moveToBottom();
    arrowsLayer.draw();
    mapLayer.draw();

    mapZoomSlider = document.getElementById("map-zoom");
    mapZoomSlider.oninput = function () {
        map_scale = parseInt(mapZoomSlider.value, 10) / 100;
        redraw_map(map_scale, robot_position.x, robot_position.y);
    }
    map_scale = parseInt(mapZoomSlider.value, 10) / 100;

    targets_table = document.getElementById('targets').getElementsByTagName('tbody')[0];
    routeStatusLabel = document.getElementById('route-status');

    loading_dropdown = document.getElementById('loading-station');
    unloading_dropdown = document.getElementById('unloading-station');

    currentPath = new Konva.Line({
        x: 0,
        y: 0,
        points: [],
        stroke: 'red',
        tension: 0
    });

    currentPosesArray = [];

    arrowsLayer.add(currentPath);

    socket.on('robot_pose', function (robot_coordinates) {
        robot_position.x = robot_coordinates.x_pos;
        robot_position.y = robot_coordinates.y_pos;
        robot_position.theta = robot_coordinates.theta;
        updateRobotArrow(robot_position.theta);
        redraw_map(map_scale, robot_position.x, robot_position.y);
    });

    socket.on('add_target', function (target) {
        for (let i = 0; i < targets_array.length; i++) {
            if (targets_array[i].id == target.id) {
                return;
            }
        }
        targets_array.push({
            id: target.id,
            x: parseFloat(target.x),
            y: parseFloat(target.y),
            theta: parseFloat(target.theta),
            label: target.label
        });
        let existing_poses_table = document.getElementById('existing_poses').getElementsByTagName('tbody')[0];
        let row = existing_poses_table.insertRow(existing_poses_table.rows.length);
        row.id = 'poseRow#' + target.id;
        row.insertCell(-1).innerHTML = "[" + target.x.toFixed(2) + ", " + target.y.toFixed(2) + ", " + (target.theta * 180 / Math.PI).toFixed(2) + "]";
        row.insertCell(-1).innerHTML = target.label;
        let buttonsCell = row.insertCell(-1);
        buttonsCell.appendChild(createAddToRouteButtonElement(target.id));
        buttonsCell.classList.add("text-center");
        addTableRow(targets_table, target);
        updateTargets();
    });

    socket.on('remove_target_by_id', function (target_id) {
        for (var i = 0; i < targets_array.length; i++) {
            if (targets_array[i].id === target_id) {
                targets_array.splice(i, 1);
            }
        }
        let labelDiv = document.getElementById('label#' + target_id);
        if (labelDiv) {
            labelDiv.parentNode.removeChild(labelDiv);
        }
        let tableRow = document.getElementById('targetRow#' + target_id);
        if (tableRow) {
            tableRow.parentNode.removeChild(tableRow);
        }
        let targetArrow = stage.find('#targetArrow#' + target_id)[0];
        if (targetArrow) {
            targetArrow.remove();
            arrowsLayer.draw();
        }
    });

    socket.on('map_update', function (map_data) {
        var blob = new Blob([map_data.blob], { type: 'image/png' });
        map_metadata = map_data.metadata;
        var url = URL.createObjectURL(blob);
        Konva.Image.fromURL(url, function (mapImageNode) {
            mapLayer.removeChildren();
            map_image_element = mapImageNode;
            redraw_map(map_scale, robot_position.x, robot_position.y);
            URL.revokeObjectURL(url);
        });
    });

    socket.on('new_route_point', function (route_point) {
        for (let i = 0; i < route_array.length; i++) {
            if (route_array[i].point_routeID == route_point.point_routeID) {
                return;
            }
        }
        console.log("Add new route point to table, sequence: " + route_point.route_sequence + ", ID: " + route_point.point_navID);
        route_array.push(route_point);
        addRouteTableRow(route_point.target, route_point.route_sequence, route_point.point_routeID);
    });

    socket.on('del_route_point', function (routeID) {
        console.log("Remove route point from table: " + routeID);
        for (var i = 0; i < route_array.length; i++) {
            if (route_array[i].point_routeID === routeID) {
                route_array.splice(i, 1);
            }
        }
        removeRouteTableRow(routeID);
    });

    socket.on('route_status', function (status) {
        console.log('route_status\n', status)
        routeStatusLabel.innerHTML = "Dest " + status.label + " [" + status.seq + "/" + status.max + "]";
    });

    socket.on('current_plan', function (plan_object) {
        redrawPath(plan_object.path.poses);
    });

    socket.on('set_scale_range', function (scale_range) {
        console.log("Map scale: [", scale_range.min, ", ", scale_range.max, "]")
        document.getElementById("map-zoom").min = scale_range.min;
        document.getElementById("map-zoom").max = scale_range.max;
    });

    socket.on('map_file_list', updateMapFilenames);
    saveCurrentPosDialog = document.getElementById("saveCurrentPosDialog");
    mapSettingsDialog = document.getElementById("mapSettingsDialog");
    uploadMapProgressBar = document.getElementById("mapUploadProgress");
    mapSettingsTab = document.getElementById("mapSettingsTab");
    mapUploadTab = document.getElementById("mapUploadTab");
    mapFileDropdown = document.getElementById("mapFileDropdown");
    mapModeRadioStatic = document.getElementById("map_mode_static");
    mapModeRadioSLAM = document.getElementById("map_mode_slam");
    mapAutoSaveCheckbox = document.getElementById("mapAutoSaveCheckbox");
    routeManagerDialog = document.getElementById("routeManagerDialog");
    pathCreatorDialog = document.getElementById("pathCreatorDialog");

    stageDiv.addEventListener("wheel", mapResizeCallback);

    var hammertime = new Hammer(stageDiv);
    hammertime.get('pinch').set({ enable: true });
    hammertime.on('pinch', mapResizeCallback);

    setTargetLabelsCheckboxListener();
    setTargetsTableCheckboxListener();
    setZoomSliderCheckboxListener();
};