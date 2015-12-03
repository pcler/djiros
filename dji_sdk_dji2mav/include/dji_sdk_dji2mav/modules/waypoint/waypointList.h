/*****************************************************************************
 * @Brief     Functional class for waypoint. Mav-free and ROS-free singleton
 * @Version   0.2.1
 * @Author    Chris Liu
 * @Created   2015/11/18
 * @Modified  2015/12/02
 *****************************************************************************/

#ifndef _DJI2MAV_WAYPOINTLIST_H_
#define _DJI2MAV_WAYPOINTLIST_H_


#include <iostream>
#include <new>
#include <stdio.h>

#define MAX_WP_LIST_SIZE 128

namespace dji2mav {

    class WaypointList {
        public:
            WaypointList() {
                m_listSize = 0;
                m_targetIdx = -1;
            }


            ~WaypointList() {
            }


            inline const float ( *getWaypointList() )[7] {
                return m_wpList;
            }


            inline const uint16_t* getCmdList() {
                return m_cmd;
            }


            inline uint16_t getListSize() {
                return m_listSize;
            }


            inline void setListSize(const uint16_t size) {
                m_listSize = size;
            }


            inline uint16_t getTargetIdx() {
                return m_targetIdx;
            }


            inline void setTargetIdx(uint16_t targetIdx) {
                m_targetIdx = targetIdx;
            }


            inline void readyToUpload() {
                m_targetIdx = 0;
            }


            inline void readyToDownload() {
                m_targetIdx = 0;
            }


            inline void finishUpload() {
                if(m_targetIdx != m_listSize) {
                    printf("Uploaded waypoint list size doesn't matched!\n");
                }
                m_targetIdx = -1;
            }


            inline bool isDownloadFinished() {
                if(m_targetIdx == m_listSize) {
                    m_targetIdx = -1;
                    return true;
                } else {
                    return false;
                }
            }


            inline bool isValidIdx(uint16_t idx) {
                return (idx < m_listSize) ? true : false;
            }


            void getWaypointData(uint16_t idx, uint16_t &cmd,
                    float &param1, float &param2, float &param3, 
                    float &param4, float &lat, float &lon, float &alt) {

                cmd = m_cmd[idx];
                param1 = m_wpList[idx][0];
                param2 = m_wpList[idx][1];
                param3 = m_wpList[idx][2];
                param4 = m_wpList[idx][3];
                lat = m_wpList[idx][4];
                lon = m_wpList[idx][5];
                alt = m_wpList[idx][6];

                ++m_targetIdx;

            }


            void setWaypointData(uint16_t idx, uint16_t cmd, float param1, 
                    float param2, float &param3, float param4, float lat, 
                    float lon, float alt) {

                m_cmd[idx] = cmd;
                m_wpList[idx][0] = param1;
                m_wpList[idx][1] = param2;
                m_wpList[idx][2] = param3;
                m_wpList[idx][3] = param4;
                m_wpList[idx][4] = lat;
                m_wpList[idx][5] = lon;
                m_wpList[idx][6] = alt;

                ++m_targetIdx;

            }


            inline uint16_t getWaypointCmd(uint16_t idx) {
                return m_cmd[idx];
            }


            inline void setWaypointCmd(uint16_t idx, uint16_t cmd) {
                m_cmd[idx] = cmd;
            }


            inline float getWaypointParam1(uint16_t idx) {
                return m_wpList[idx][0];
            }


            inline void setWaypointParam1(uint16_t idx, float param1) {
                m_wpList[idx][0] = param1;
            }


            inline float getWaypointParam2(uint16_t idx) {
                return m_wpList[idx][1];
            }


            inline void setWaypointParam2(uint16_t idx, float param2) {
                m_wpList[idx][1] = param2;
            }


            inline float getWaypointParam3(uint16_t idx) {
                return m_wpList[idx][2];
            }


            inline void setWaypointParam3(uint16_t idx, float param3) {
                m_wpList[idx][2] = param3;
            }


            inline float getWaypointParam4(uint16_t idx) {
                return m_wpList[idx][3];
            }


            inline void setWaypointParam4(uint16_t idx, float param4) {
                m_wpList[idx][3] = param4;
            }


            inline float getWaypointLat(uint16_t idx) {
                return m_wpList[idx][4];
            }


            inline void setWaypointLat(uint16_t idx, float lat) {
                m_wpList[idx][4] = lat;
            }


            inline float getWaypointLon(uint16_t idx) {
                return m_wpList[idx][5];
            }


            inline void setWaypointLon(uint16_t idx, float lon) {
                m_wpList[idx][5] = lon;
            }


            inline float getWaypointAlt(uint16_t idx) {
                return m_wpList[idx][5];
            }


            inline void setWaypointAlt(uint16_t idx, float alt) {
                m_wpList[idx][5] = alt;
            }


            inline float getWpHeading(uint16_t idx) {
                return getWaypointParam4(idx);
            }


            inline void setWpHeading(uint16_t idx, float heading) {
                setWpHeading(idx, heading);
            }


            inline float getWpStaytime(uint16_t idx) {
                return getWaypointParam1(idx);
            }


            inline void setWpStaytime(uint16_t idx, float staytime) {
                setWaypointParam1(idx, staytime);
            }


            bool clearMission() {
                m_listSize = 0;
                m_targetIdx = -1;
            }


            void displayMission() {
                printf("Display the full mission:\n");
                for(uint16_t i = 0; i < m_listSize; ++i) {
                    printf("Index %d: param1 %f, param2 %f, param3 %f, "
                            "param4 %f, lat %f, lon %f, alt %f\n", i, 
                            m_wpList[i][0], m_wpList[i][1], m_wpList[i][2], 
                            m_wpList[i][3], m_wpList[i][4], m_wpList[i][5], 
                            m_wpList[i][6]);
                }
                printf("--- End of display ---\n\n");
            }


        private:
            float m_wpList[MAX_WP_LIST_SIZE][7];
            uint16_t m_cmd[MAX_WP_LIST_SIZE];
            uint16_t m_listSize;
            int m_targetIdx;

    };

}


#endif
