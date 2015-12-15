/****************************************************************************
 * @Brief   A bringup node for dji2mav. Using dji2mav interface v0.2.x
 * @Version 0.2.1
 * @Author  Chris Liu
 * @Create  2015/11/02
 * @Modify  2015/12/04
 ****************************************************************************/

#include <pthread.h>
#include <string>

#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/Velocity.h>
#include <dji_sdk/AttitudeQuaternion.h>

#include "dji_sdk_dji2mav/config.h"



DJIDrone* drone;



/* A thread for sending heartbeat */
void* sendHb_Period(void* args) {
    int t_s = *((int*) args);
    while( ros::ok() ) {
        dji2mav::MavHeartbeat::getInstance()->sendHeartbeat();
        sleep(t_s);
    }
}

/* A thread for sending sensor data */
void* sendSs_Period(void* args) {
    int t_ms = *((int*) args);
    while( ros::ok() ) {
        dji2mav::MavSensors::getInstance()->sendSensorsData();
        usleep(t_ms);
    }
}



void locPosCB(const dji_sdk::LocalPosition &msg) {
    dji2mav::MavSensors::getInstance()->setLocalPosition(&msg.ts, &msg.x, 
            &msg.y, &msg.z);
}

void velCB(const dji_sdk::Velocity &msg) {
    dji2mav::MavSensors::getInstance()->setVelocity(&msg.ts, &msg.vx, 
            &msg.vy, &msg.vz);
}

void attCB(const dji_sdk::AttitudeQuaternion &msg) {
    dji2mav::MavSensors::getInstance()->setAttitudeQuaternion(&msg.ts, 
            &msg.q0, &msg.q1, &msg.q2, &msg.q3, &msg.wx, &msg.wy, &msg.wz);
}

void gloPosCB(const dji_sdk::GlobalPosition &msg) {
    dji2mav::MavSensors::getInstance()->setGlobalPosition(&msg.ts, 
            &msg.latitude, &msg.longitude, &msg.altitude, &msg.height);
}



void respondToHeartbeat() {
    ROS_INFO("Get heartbeat\n");
}

void respondToMissionRequestList() {
    ROS_INFO("Get mission request list\n");
}

void respondToMissionRequest(uint16_t param) {
    ROS_INFO("Get mission request %d", param);
}

void respondToMissionAck() {
    ROS_INFO("Mission ack get\n");
}

void respondToMissionCount(uint16_t param) {
    ROS_INFO("Get mission count %d", param);
}

void respondToMissionItem(uint16_t param) {
    ROS_INFO("Get mission item %d", param);
}

void respondToMissionClearAll() {
    ROS_INFO("Get mission clear all");
}

void respondToMissionSetCurrent(uint16_t param) {
    ROS_INFO("Get mission set current %u", param);
}



void respondToTarget(const float mission[][7], uint16_t beginIdx, 
        uint16_t endIdx) {

    dji_sdk::MissionWaypointTask task;
    memset(&task, 0, sizeof(dji_sdk::MissionWaypointTask));

    task.mission_exec_times = 0x01;
    task.yaw_mode = 0x03;
    task.velocity_range = 10.0; //must be set
    task.idle_velocity = 3.0; //must be set

    ROS_INFO("beginIdx %d, endIdx %d", beginIdx, endIdx);
    for(int i = beginIdx; i < endIdx; ++i) {
        dji_sdk::MissionWaypoint wp;
        memset(&wp, 0, sizeof(dji_sdk::MissionWaypoint));

        wp.latitude = mission[i][4];
        wp.longitude = mission[i][5];
        wp.altitude = mission[i][6];
        wp.target_yaw = (int16_t)mission[i][3];
        wp.has_action = 0x01; //true or false
        wp.action_time_limit = 0xffff;

        wp.waypoint_action.action_repeat = 0x01; //times!
        wp.waypoint_action.command_list[0] = 0x00;
        wp.waypoint_action.command_parameter[0] = (int16_t) (mission[i][0] * 1000.0);

        task.mission_waypoint.push_back(wp);
    }
    ROS_INFO("Size of the wpl: %d", task.mission_waypoint.size());

    /**
     * Currently this is executed in main thread, So don't wait for server or 
     * result in case of blocking the distribution process. A new coming 
     * version will bring up a better architecture soon
     */
/*
    ROS_INFO("Going to wait for server...");
    drone->waypoint_navigation_wait_server();
*/

    drone->mission_waypoint_upload(task);
    ros::Duration(1.0).sleep();
    drone->mission_start();

/*
    ROS_INFO("Going to wait for result");
    if(drone->waypoint_navigation_wait_for_result()) {
        ROS_INFO("Succeed to execute current task!");
    } else {
        ROS_INFO("Fail to execute current task in 10 seconds!");
    }
*/

}


void respondToHpTarget(const float hp[], uint16_t size, uint16_t cmd) {
    dji_sdk::MissionHotpointTask task;
    memset(&task, 0, sizeof(dji_sdk::MissionHotpointTask));

    task.latitude = hp[4];
    task.longitude = hp[5];
    task.altitude = hp[6];

    if(hp[2] < 0)
        task.is_clockwise = 0x00;
    else
        task.is_clockwise = 0x01;
    task.radius = abs(hp[2]);

    //task.? = hp[3];//unused

    task.angular_speed = 5;
    task.start_point = 0x04;//frome current position to the nearest point
    task.yaw_mode = 0x01;//point to the center of the circle

    switch(cmd) {
        case 17:
            break;
        case 18:
            //turns
            break;
        case 19:
            //time
            break;
    }

    drone->mission_hotpoint_upload(task);
    ros::Duration(1.0).sleep();
    drone->mission_start();
}



int main(int argc, char* argv[]) {

    ros::init(argc, argv, "dji2mav_bringup");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    drone = new DJIDrone(nh);

    std::string targetIp1;
    int targetPort1;
    int srcPort;
    nh_private.param( "targetIp1", targetIp1, std::string("10.60.23.136") );
    nh_private.param("targetPort1", targetPort1, 14550);
    nh_private.param("srcPort", srcPort, 14551);

    drone->activate();
    ros::Duration(1.0).sleep();
    drone->request_sdk_permission_control();

    dji2mav::Config* config = dji2mav::Config::getInstance();
    /* set the sysid "1" and the number of GCS is also "1" */
    config->setup(1, 1);
    /* The index of first GCS is "0" */
    config->start(0, targetIp1, (uint16_t)targetPort1, (uint16_t)srcPort);

    /* Register Subscribers */
    ros::Subscriber sub1 = nh.subscribe("/dji_sdk/local_position", 1, locPosCB);
    ros::Subscriber sub2 = nh.subscribe("/dji_sdk/velocity", 1, velCB);
    ros::Subscriber sub3 = nh.subscribe("/dji_sdk/attitude_quaternion", 1, attCB);
    ros::Subscriber sub4 = nh.subscribe("/dji_sdk/global_position", 1, gloPosCB);

    /* Heartbeat send thread */
    pthread_t hbTid;
    int hb_sec = 1;
    int hb_thread_ret = pthread_create(&hbTid, NULL, sendHb_Period, (void*)&hb_sec);
    if(0 != hb_thread_ret) {
        ROS_ERROR("Create pthread for sending heartbeat fail! Error code: %d", hb_thread_ret);
    }

    /* Sensors data send thread */
    pthread_t ssTid;
    int ss_sec = 20000;
    int ss_thread_ret = pthread_create(&ssTid, NULL, sendSs_Period, (void*)&ss_sec);
    if(0 != ss_thread_ret) {
        ROS_ERROR("Create pthread for sending sensors data fail! Error code: %d", ss_thread_ret);
    }

    /* Register responser */
//  dji2mav::MavHeartbeat::getInstance()->setHeartbeatRsp(respondToHeartbeat);
    dji2mav::MavWaypoint::getInstance()->setMissionRequestListRsp(respondToMissionRequestList);
    dji2mav::MavWaypoint::getInstance()->setMissionRequestRsp(respondToMissionRequest);
    dji2mav::MavWaypoint::getInstance()->setMissionAckRsp(respondToMissionAck);
    dji2mav::MavWaypoint::getInstance()->setMissionCountRsp(respondToMissionCount);
    dji2mav::MavWaypoint::getInstance()->setMissionItemRsp(respondToMissionItem);
    dji2mav::MavWaypoint::getInstance()->setMissionClearAllRsp(respondToMissionClearAll);
    dji2mav::MavWaypoint::getInstance()->setMissionSetCurrentRsp(respondToMissionSetCurrent);
    dji2mav::MavWaypoint::getInstance()->setTargetRsp(respondToTarget);

    dji2mav::MavDistributor::getInstance()->m_moduleHp->run();
    dji2mav::MavDistributor::getInstance()->m_moduleHp->setTargetRsp(respondToHpTarget);

    while( ros::ok() ) {
        /* Do distribution in loop */
        dji2mav::MavDistributor::getInstance()->distribute();

        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    ROS_INFO("Going to distruct the whole process...");
    drone->release_sdk_permission_control();
    config->distructor();
    delete drone;

    return 0;
}

