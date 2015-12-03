/*****************************************************************************
 * @Brief     Provide static conversion methods. ROS-depended and mav-depended
 * @Version   0.2.1
 * @Author    Chris Liu
 * @Created   2015/12/02
 * @Modified  2015/12/02
 *****************************************************************************/

#ifndef _DJI2MAV_WAYPOINTADAPTER_H_
#define _DJI2MAV_WAYPOINTADAPTER_H_


//#include <dji_sdk/dji_drone.h>
#include <dji_sdk/MissionWaypointTask.h>

#include <mavlink/common/mavlink.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

namespace dji2mav{

    class WaypointAdapter{
        public:
            /**
             * @brief  Convert std datatype to SDK ROS mission type
             * @param  mission : A 2 dimensions array of mission data
             * @param  cmd     : Corresponding command code
             * @return A SDK ROS mission type that is converted
             */
            static dji_sdk::MissionWaypointTask convert(DJIDrone* drone, 
                    const float mission[][7], uint16_t missionSize, 
                    const uint16_t cmd[], uint8_t times) {

                dji_sdk::MissionWaypointTask task;
                memset(&task, 0, sizeof(dji_sdk::MissionWaypointTask));

                //task.velocity_range = ;
                //task.idle_velocity = ;
                //task.action_on_finish = ;
                task.mission_exec_times = times;
                //task.yaw_mode = ;
                //task.trace_mode = ;
                //task.action_on_rc_lost = ;
                //task.gimbal_pitch_mode = ;

                for(uint16_t i = 0; i < missionSize; ++i) {
                    dji_sdk::MissionWaypoint wp;
                    memset(&wp, 0, sizeof(dji_sdk::MissionWaypoint));

                    wp.latitude = mission[i][4];
                    wp.longitude = mission[i][5];
                    wp.altitude = mission[i][6];

                    switch(cmd[i]) {
                        case MAV_CMD_NAV_WAYPOINT:
                            wp.damping_distance = abs(mission[i][2]);
                            wp.turn_mode = mission[i][2] >= 0? 0x00 : 0x01;

                            wp.target_yaw = (int16_t)mission[i][3];
                            //wp.target_gimbal_pitch = ;

                            wp.has_action = 0x01; //true or false
                            wp.action_time_limit = 0xffff; //manually add

                            //wp.waypoint_action.action_repeat = ;
                            wp.waypoint_action.command_list[0] = 0; //stay
                            wp.waypoint_action.command_parameter[0] = 
                                    mission[i][0]; //staytime
                            break;
                        default:
                            printf("It's an unknown command #%u!\n", cmd[i]);
                            break;
                    }

                    task.mission_waypoint.push_back(wp);
                }

                return task;

            }

    };

} // namespace dji2mav


#endif
