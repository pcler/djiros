var curr_lat = 0;
var curr_lon = 0;
var curr_hei = 0;

var sdk_permission = -1;


function Communicator(socket) {

    //rosbridge config
    this.ros = new ROSLIB.Ros({
        url : socket.url
    });

    this.ros.on('connection', function(){
        console.log('Connected to websocket server.');
        $( '<div>Succeed to connect to websocket server.</div>' ).appendTo("#monitor");
    });
    this.ros.on('error', function(){
        console.log('Error connecting to websocket server: ', error);
        $( '<div>Error connecting to websocket server: '+error+'</div>' ).appendTo("#monitor");
    });
    this.ros.on('close', function(){
        console.log('Connection is closed!.');
        $( '<div>Connection is closed!</div>' ).appendTo("#monitor");
    });

    //service client
    this.permissionCtrlClient = new ROSLIB.Service({
        ros : this.ros,
        name : 'dji_sdk/sdk_permission_control',
        serviceType : 'dji_sdk/SDKPermissionControl'
    });

    this.droneArmCtrlClient = new ROSLIB.Service({
        ros : this.ros,
        name : 'dji_sdk/drone_arm_control',
        serviceType : 'dji_sdk/DroneArmControl'
    });

    this.missionWpUploadClient = new ROSLIB.Service({
        ros : this.ros,
        name : 'dji_sdk/mission_waypoint_upload',
        serviceType : 'dji_sdk/MissionWpUpload'
    });

    this.missionDownloadClient = new ROSLIB.Service({
        ros : this.ros,
        name : 'dji_sdk/mission_download',
        serviceType : 'dji_sdk/MissionDownload'
    });

    this.missionStartClient = new ROSLIB.Service({
        ros : this.ros,
        name : 'dji_sdk/mission_start',
        serviceType : 'dji_sdk/MissionStart'
    });

    this.missionPauseClient = new ROSLIB.Service({
        ros : this.ros,
        name : 'dji_sdk/mission_pause',
        serviceType : 'dji_sdk/MissionPause'
    });

    this.missionCancelClient = new ROSLIB.Service({
        ros : this.ros,
        name : 'dji_sdk/mission_cancel',
        serviceType : 'dji_sdk/MissionCancel'
    });
    //more service can be added!


    //subscriber
    this.globalPosListener = new ROSLIB.Topic({
        ros : this.ros,
        name : 'dji_sdk/global_position',
        messageType : 'dji_sdk/GlobalPosition'
    });
    this.globalPosListener.subscribe(function(msg) {
        curr_lat = msg.latitude;
        curr_lon = msg.longitude;
        curr_hei = msg.height;
    });
    /*var globalPosTimer = setInterval(function() {
        console.log('Current pos: ' + curr_lat + ', ' + curr_lon + ', ' + curr_hei);
    }, 1000);*/

    this.sdkPermissionListener = new ROSLIB.Topic({
        ros : this.ros,
        name : 'dji_sdk/sdk_permission',
        messageType : 'std_msgs/UInt8'
    });
    this.sdkPermissionListener.subscribe(function(msg) {
        sdk_permission = msg.data;
    });
}

Communicator.prototype.getGlobalPos = function() {
    return [curr_lat, curr_lon, curr_hei];
}

Communicator.prototype.setNavigationMode = function() {

    var _req = new ROSLIB.ServiceRequest({
        control_enable : 1
    });

    console.log('Request to obtain control...');
    $( '<div>Request to obtain control</div>' ).appendTo("#monitor");
    this.permissionCtrlClient.callService(_req, function(result) {
        setTimeout(function() {
            if(1 == sdk_permission)
                console.log('...succeed to obtain control');
            else if(2 == sdk_permission)
                console.log('...fail to obtain control');
            else
                console.log('...unknown error');
        }, 40);
    });

};

Communicator.prototype.stopNavigationMode = function() {

    var _req = new ROSLIB.ServiceRequest({
        control_enable : 0
    });

    console.log('Request to release control...');
    $( '<div>Request to release control</div>' ).appendTo("#monitor");
    this.permissionCtrlClient.callService(_req, function(result) {
        setTimeout(function() {
            if(0 == sdk_permission)
                console.log('...succeed to release control');
            else if(3 == sdk_permission)
                console.log('...fail to release control');
            else
                console.log('...unknown error');
        }, 40);
    });

};

Communicator.prototype.uploadWayline = function() {

    if (my_planner.markerList.length < 2){
        alert('Please set 2 waypoints at least!');
        return;
    }

    // rosbridge config
    console.log('Generate new waypoint mission');

    var _missionWpList = new Array();
    for(i = 0; i < my_planner.markerList.length; i++) {
        var _action = new ROSLIB.Message({
            action_repeat : 1,
            command_list : new Array(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
            command_parameter : new Array(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)//staytime etc
        });
        var _wp = new ROSLIB.Message({
            latitude : my_planner.markerList[i][1].point.lat,
            longitude : my_planner.markerList[i][1].point.lng,
            altitude : my_planner.markerList[i][2].alti,
            damping_distance : 0.2,//0.2m
            target_yaw : 0,//unset
            turn_mode : 0,//clockwise
            has_action : 1,//true or false
            action_time_limit : 65535,//0xffff, unsigned int16
            waypoint_action : _action
        });
        _missionWpList.push(_wp);
    }
    var _missionWpTask = new ROSLIB.Message({
        velocity_range : 10,//max speed
        idle_velocity : 5,//idle speed
        action_on_finish : 0,//do nothing
        mission_exec_times : 1,//exec this mission 1 time
        yaw_mode : 0,//auto
        trace_mode : 0,//point to point
        action_on_rc_lost : 0,//quit waypoint
        gimbal_pitch_mode : 0,//free
        mission_waypoint : _missionWpList
    });
    var _req = new ROSLIB.ServiceRequest({
        waypoint_task : _missionWpTask
    });
    console.log('Upload waypoint mission to server...');
    $( '<div>Uploading waypoint mission. Please wait...</div>' ).appendTo("#monitor");
    this.missionWpUploadClient.callService(_req, function(result) {
        console.log(result);
        if(true == result) {
            console.log('...succeed to upload waypoint mission');
            $( '<div>...succeed to upload waypoint mission!</div>' ).appendTo("#monitor");
        } else {
            console.log('...fail to upload waypoint mission');
            //there is a respond defect in mission_upload_server
            $( '<div>...finish uploading waypoint mission!</div>' ).appendTo("#monitor");
        }
    });

};


Communicator.prototype.startWayline = function() {

    var _req = new ROSLIB.ServiceRequest({
        //empty request
    });

    console.log('Request to start the waypoint mission...');
    $( '<div>Request to start the waypoint mission</div>' ).appendTo("#monitor");
    this.missionStartClient.callService(_req, function(result) {
        //there is a respond defect in mission_upload_server
        console.log('...finish sending start command for the waypoint mission');
        /*if(true == result.result)
            console.log('...succeed to start the waypoint mission');
        else
            console.log('...fail to start the waypoint mission');*/
    });

};

Communicator.prototype.cancelWayline = function() {

    var _req = new ROSLIB.ServiceRequest({
        //empty request
    });

    console.log('Request to cancel the waypoint mission...');
    $( '<div>Request to cancel the waypoint mission</div>' ).appendTo("#monitor");
    this.missionCancelClient.callService(_req, function(result) {
        //there is a respond defect in mission_upload_server
        console.log('...finish sending cancel command for the waypoint mission');
        /*if(true == result.result)
            console.log('...succeed to cancel the waypoint mission');
        else
            console.log('...fail to cancel the waypoint mission');*/
    });

};

Communicator.prototype.pauseWayline = function() {

    var _req = new ROSLIB.ServiceRequest({
        pause : 0
    });

    console.log('Request to pause the waypoint mission...');
    $( '<div>Request to pause the waypoint mission</div>' ).appendTo("#monitor");
    this.missionPauseClient.callService(_req, function(result) {
        //there is a respond defect in mission_upload_server
        console.log('...finish sending pause command for the waypoint mission');
        /*if(true == result.result)
            console.log('...succeed to pause the waypoint mission');
        else
            console.log('...fail to pause the waypoint mission');*/
    });

};

//TODO: have't implemented this function in ROS yet
Communicator.prototype.continueWayline = function() {

    var _req = new ROSLIB.ServiceRequest({
        pause : 1
    });

    console.log('Request to start the waypoint mission...');
    $( '<div>Request to resume the waypoint mission</div>' ).appendTo("#monitor");
    this.missionPauseClient.callService(_req, function(result) {
        //there is a respond defect in mission_upload_server
        console.log('...finish sending resume command for the waypoint mission');
        /*if(true == result.result)
            console.log('...succeed to start the waypoint mission');
        else
            console.log('...fail to start the waypoint mission');*/
    });

};
