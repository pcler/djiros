/*****************************************************************************
 * @Brief     Module base class that provide common methods. ROS-free
 * @Version   0.2.2
 * @Author    Chris Liu
 * @Created   2015/12/03
 * @Modified  2015/12/03
 *****************************************************************************/

#ifndef _MAV2DJI_MAVMODULE_H_
#define _MAV2DJI_MAVMODULE_H_


#include <iostream>
#include <stdio.h>
#include <new>
#include <assert.h>
#include <limits.h>

namespace dji2mav{

    class MavWaypoint {
        public:
            /**
             * @brief  Get the senderRecord array address
             * @return The address of senderRecord
             */
            inline const int* getSenderRecord() {
                return m_senderRecord;
            }


            /**
             * @brief  Set the sender index of specific GCS
             * @param  gcsIdx    : The index of GCS
             * @param  senderIdx : The index of sender
             * @return True if succeed or false if fail
             */
            bool setSenderIdx(uint16_t gcsIdx, int senderIdx) {
                if( !m_hdlr->isValidIdx(gcsIdx, senderIdx) )
                    return false;
                m_senderRecord[gcsIdx] = senderIdx;
                return true;
            }


            /**
             * @brief  Set the master with its gcsIdx and senderIdx
             * @param  gcsIdx    : The index of GCS that is to be set
             * @param  senderIdx : The index of sender of GCS
             * @return True if succeed or false if fail
             */
            bool setMasterGcsIdx(uint16_t gcsIdx, uint16_t senderIdx) {
                if( m_hdlr->isValidIdx(gcsIdx, senderIdx) ) {
                    m_senderRecord[m_masterGcsIdx] = -1;
                    m_masterGcsIdx = gcsIdx;
                    m_senderRecord[m_masterGcsIdx] = senderIdx;
                    return true;
                } else {
                    printf("Invalid master GCS index %u "
                            "and sender index %u.\n", gcsIdx, senderIdx);
                    return false;
                }
            }


            /**
             * @brief  Register new sender and use it for specific GCS
             * @param  gcsIdx : The index of GCS
             * @return True if succeed or false if fail
             */
            bool applyNewSender(uint16_t gcsIdx) {
                int newSender = m_hdlr->registerSender(gcsIdx);
                if( newSender < 0 ) {
                    printf("Fail to regiser sender for waypoint in GCS #%u! "
                            "Did you set sender list too small?\n", gcsIdx);
                    return false;
                }
                m_senderRecord[gcsIdx] = newSender;
                return true;
            }


            /**
             * @brief  Register new sender and use it for master GCS
             * @return True if succeed or false if fail
             */
            inline bool applyNewSender() {
                return applyNewSender(m_masterGcsIdx);
            }


            /**
             * @brief Use the general sender for specific GCS
             * @param gcsIdx : The index of GCS
             */
            inline void applyGeneralSender(uint16_t gcsIdx) {
                m_senderRecord[gcsIdx] = m_hdlr->getGeneralSenderIdx(gcsIdx);
            }


            /**
             * @brief Use the general sender for master GCS
             */
            inline void applyGeneralSender() {
                applyGeneralSender(m_masterGcsIdx);
            }


            void distructor() {
                delete m_instance;
            }


        private:
            MavWaypoint() {

                assert(CHAR_BIT * sizeof(float) == 32);
                assert(CHAR_BIT * sizeof(double) == 64);

                m_hdlr = MavHandler::getInstance();

                m_masterGcsIdx = 0;

                try {
                    m_senderRecord = new int[m_hdlr->getMngListSize()];
                    memset( m_senderRecord, 0, 
                            m_hdlr->getMngListSize() * sizeof(int) );
                } catch(std::bad_alloc& m) {
                    std::cerr << "Failed to alloc memory for senderRecord: " 
                            << "at line: " << __LINE__ << ", func: " 
                            << __func__ << ", file: " << __FILE__ 
                            << std::endl;
                    perror( m.what() );
                    exit(EXIT_FAILURE);
                }

                // default #0 GCS send the waypoint cmd to the vehicle
                setMasterGcsIdx(0, m_hdlr->getGeneralSenderIdx(0));

                setMissionRequestListRsp(NULL);
                setMissionRequestRsp(NULL);
                setMissionAckRsp(NULL);
                setMissionCountRsp(NULL);
                setMissionItemRsp(NULL);

                printf("Succeed to construct Waypoint module\n");

            }


            ~MavWaypoint() {
                if(m_senderRecord != NULL) {
                    delete []m_senderRecord;
                    m_senderRecord = NULL;
                }
                m_hdlr = NULL;
                printf("Finish destructing Waypoint module\n");
            }


            static MavWaypoint* m_instance;
            int* m_senderRecord;
            int m_masterGcsIdx;

            MavHandler* m_hdlr;
            WaypointList m_wpl;

            mavlink_message_t m_sendMsg;
            mavlink_mission_request_t m_reqMsg;
            mavlink_mission_count_t m_cntMsg;
            mavlink_mission_ack_t m_ackMsg;
            mavlink_mission_item_t m_itemMsg;
            mavlink_mission_set_current_t m_setCurrMsg;
            mavlink_mission_current_t m_currMsg;

            enum {
                idle, 
                uploading, 
                downloading, 
                loaded, 
                executing, 
                paused, 
                error
            } m_status;

            void (*m_missionRequestListRsp)();
            void (*m_missionRequestRsp)(uint16_t);
            void (*m_missionAckRsp)();
            void (*m_missionCountRsp)(uint16_t);
            void (*m_missionItemRsp)(uint16_t);
            void (*m_missionClearAllRsp)();
            void (*m_missionSetCurrentRsp)(uint16_t);
            void (*m_targetRsp)(const float[][7], uint16_t, uint16_t);


    };

    MavWaypoint* MavWaypoint::m_instance = NULL;

} //namespace dji2mav


#endif
