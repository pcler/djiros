/*****************************************************************************
 * @Brief     Hotpoint module. Implement hotpoint protocol here 
 * @Version   0.2.2
 * @Author    Chris Liu
 * @Created   2015/12/11
 * @Modified  2015/12/11
 *****************************************************************************/

#ifndef _DJI2MAV_MAVHOTPOINT_H_
#define _DJI2MAV_MAVHOTPOINT_H_


#include "dji_sdk_dji2mav/modules/mavModule.h"
#include "dji_sdk_dji2mav/mavHandler.h"
#include "hotpointData.h"

#include <mavlink/common/mavlink.h>
#include <iostream>
#include <stdio.h>
#include <new>
#include <pthread.h>

namespace dji2mav{

    class MavHotpoint : public MavModule {
        public:
            MavHotpoint() : MavModule(4096) {
                //default use #0 GCS to perform hotpoint
                setMasterGcsIdx(0);

                m_hdlr = MavHandler::getInstance();

                setMissionRequestListRsp(NULL);
                setMissionRequestRsp(NULL);
                setMissionAckRsp(NULL);
                setMissionCountRsp(NULL);
                setMissionItemRsp(NULL);
                setMissionClearAllRsp(NULL);
                setMissionSetCurrentRsp(NULL);

                printf("Succeed to construct Hotpoint module\n");
            }


            ~MavHotpoint() {
            }


            /**
             * @brief  Implement the message delivery function for hotpoint
             */
            void deliverMsg() {
                mavlink_message_t msg;
                while(true) {
                    if( pullMsg(msg) ) {
                        switch(msg.msgid) {
                            //TODO: shrink the processors?
                            case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
                                reactToMissionRequestList(getMasterGcsIdx(), 
                                        &msg);
                                break;
                            case MAVLINK_MSG_ID_MISSION_REQUEST:
                                reactToMissionRequest(getMasterGcsIdx(), &msg);
                                break;
                            case MAVLINK_MSG_ID_MISSION_ACK:
                                reactToMissionAck(getMasterGcsIdx(), &msg);
                                break;
                            case MAVLINK_MSG_ID_MISSION_COUNT:
                                reactToMissionCount(getMasterGcsIdx(), &msg);
                                break;
                            case MAVLINK_MSG_ID_MISSION_ITEM:
                                reactToMissionItem(getMasterGcsIdx(), &msg);
                                break;
                            case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
                                reactToMissionClearAll(getMasterGcsIdx(), 
                                        &msg);
                                break;
                            case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
                                reactToMissionSetCurrent(getMasterGcsIdx(), 
                                        &msg);
                                break;
                            default:
                                printf("No execution for msgid #%u in "
                                        "hotpoint module\n", msg.msgid);
                                break;
                        } // switch end
                    } // if end
                    usleep(20000); //50Hz
                } // while end
            }


            void reactToMissionRequestList(uint16_t gcsIdx, 
                    const mavlink_message_t* recvMsgPtr) {

                if(recvMsgPtr->compid != MAV_COMP_ID_MISSIONPLANNER) {
                    printf("WARNNING! The compid is %u\n", recvMsgPtr->compid);
                }

                printf("In mission request list with status: %d\n", (int)m_status);

                switch(m_status) {
                    case idle:
                    case downloading:
                    case executing:
                    case paused:
                        return;
                    case loaded:
                        m_status = uploading;
                        break;
                    case uploading:
                    case error:
                        break;
                }

                mavlink_mission_count_t cntMsg;
                cntMsg.target_system = recvMsgPtr->sysid;//TODO: why we use ptr but not a reference? longlonglong
                cntMsg.target_component = recvMsgPtr->compid;
                cntMsg.count = 1;//one single unit in the list

                mavlink_message_t sendMsg;
                mavlink_msg_mission_count_encode(m_hdlr->getSysid(), 
                        MAV_COMP_ID_MISSIONPLANNER, &sendMsg, &cntMsg);
                sendMsgToMaster(sendMsg);

                if(NULL != m_missionRequestListRsp)
                    m_missionRequestListRsp();

            }


            void reactToMissionRequest(uint16_t gcsIdx, 
                    const mavlink_message_t* recvMsgPtr) {

                if(recvMsgPtr->compid != MAV_COMP_ID_MISSIONPLANNER) {
                    printf("WARNNING! The compid is %u\n", recvMsgPtr->compid);
                }

                printf("In mission request with status: %d\n", (int)m_status);

                switch(m_status) {
                    case idle:
                    case downloading:
                    case loaded:
                    case executing:
                    case paused:
                        return;
                    case uploading:
                    case error:
                        break;
                }

                mavlink_mission_request_t reqMsg;
                mavlink_msg_mission_request_decode(recvMsgPtr, &reqMsg);

                mavlink_mission_item_t itemMsg;
                if(reqMsg.seq == 0) {
                    itemMsg.target_system = recvMsgPtr->sysid;
                    itemMsg.target_component = recvMsgPtr->compid;
                    itemMsg.seq = reqMsg.seq;
                    m_hp.getHotpointData(itemMsg.seq, itemMsg.command, 
                            itemMsg.param1, itemMsg.param2, itemMsg.param3, 
                            itemMsg.param4, itemMsg.x, itemMsg.y, itemMsg.z);
                } else {
                    printf("Invalid index of hotpoint in hotpoint module!\n");
                    return;
                }

                mavlink_message_t sendMsg;
                mavlink_msg_mission_item_encode(m_hdlr->getSysid(), 
                        MAV_COMP_ID_MISSIONPLANNER, &sendMsg, &itemMsg);
                sendMsgToMaster(sendMsg);

                if(NULL != m_missionRequestRsp)
                    m_missionRequestRsp(reqMsg.seq);

            }


            void reactToMissionAck(uint16_t gcsIdx, 
                    const mavlink_message_t* recvMsgPtr) {

                if(recvMsgPtr->compid != MAV_COMP_ID_MISSIONPLANNER) {
                    printf("WARNNING! The compid is %u\n", recvMsgPtr->compid);
                }

                printf("In mission ack with status: %d\n", (int)m_status);

                switch(m_status) {
                    case idle:
                    case downloading:
                    case loaded:
                    case executing:
                    case paused:
                        return;
                    case uploading:
                        m_status = loaded;
                        break;
                    case error:
                        break;
                }

                mavlink_mission_ack_t ackMsg;
                mavlink_msg_mission_ack_decode(recvMsgPtr, &ackMsg);
                printf("Mission ACK code: %d\n", ackMsg.type);
                m_status = loaded;
                m_hp.display();

                if(NULL != m_missionAckRsp)
                    m_missionAckRsp();

            }


            void reactToMissionCount(uint16_t gcsIdx, 
                    const mavlink_message_t* recvMsgPtr) {

                if(recvMsgPtr->compid != MAV_COMP_ID_MISSIONPLANNER) {
                    printf("WARNNING! The compid is %u\n", recvMsgPtr->compid);
                }

                printf("In mission count with status: %d\n", (int)m_status);

                switch(m_status) {
                    case idle:
                    case loaded:
                        m_status = downloading;
                        break;
                    case downloading:
                    case uploading:
                    case executing:
                    case paused:
                    case error:
                        return;
                }

                mavlink_mission_count_t cntMsg;
                mavlink_msg_mission_count_decode(recvMsgPtr, &cntMsg);

                mavlink_mission_request_t reqMsg;
                reqMsg.target_system = recvMsgPtr->sysid;
                reqMsg.target_component = recvMsgPtr->compid;
                reqMsg.seq = 0;

                mavlink_message_t sendMsg;
                mavlink_msg_mission_request_encode(m_hdlr->getSysid(), 
                        MAV_COMP_ID_MISSIONPLANNER, &sendMsg, &reqMsg);
                sendMsgToMaster(sendMsg);

                if(NULL != m_missionCountRsp)
                    m_missionCountRsp(cntMsg.count);

            }


            void reactToMissionItem(uint16_t gcsIdx, 
                    const mavlink_message_t* recvMsgPtr) {

                if(recvMsgPtr->compid != MAV_COMP_ID_MISSIONPLANNER) {
                    printf("WARNNING! The compid is %u\n", recvMsgPtr->compid);
                }

                printf("In mission item with status: %d\n", (int)m_status);

                switch(m_status) {
                    case loaded:
                    case executing:
                    case idle:
                    case uploading:
                    case paused:
                    case error:
                        return;
                    case downloading:
                        break;
                }

                mavlink_mission_item_t itemMsg;
                mavlink_msg_mission_item_decode(recvMsgPtr, &itemMsg);
                if(itemMsg.seq == 0) {
                    m_hp.setHotpointData(itemMsg.seq, itemMsg.command, 
                            itemMsg.param1, itemMsg.param2, itemMsg.param3, 
                            itemMsg.param4, itemMsg.x, itemMsg.y, itemMsg.z);
                } else {
                    printf("Invalid index of waypoint in waypoint module!\n");
                    return;
                }
printf(">>>  Mission Item: \ntarget_system: %u, \ntarget_component: %u, \nseq: %u, \nframe: %u, \ncommand: %u, \ncurrent: %u, \nautocontinue: %u, \nparam1: %f, \nparam2: %f, \nparam3: %f, \nparam4: %f, \nx: %f, \ny: %f, \nz: %f \n\n", itemMsg.target_system, itemMsg.target_component, itemMsg.seq, itemMsg.frame, itemMsg.command, itemMsg.current, itemMsg.autocontinue, itemMsg.param1, itemMsg.param2, itemMsg.param3, itemMsg.param4, itemMsg.x, itemMsg.y, itemMsg.z);

                mavlink_message_t sendMsg;
                mavlink_msg_mission_ack_pack(m_hdlr->getSysid(), 
                        MAV_COMP_ID_MISSIONPLANNER, &sendMsg, 
                        recvMsgPtr->sysid, recvMsgPtr->compid, 
                        MAV_MISSION_ACCEPTED);
                sendMsgToMaster(sendMsg);
                m_status = loaded;
                m_hp.display();

                if(NULL != m_missionItemRsp)
                    m_missionItemRsp(itemMsg.seq);

            }


            void reactToMissionClearAll(uint16_t gcsIdx, 
                    const mavlink_message_t* recvMsgPtr) {

                if(recvMsgPtr->compid != MAV_COMP_ID_MISSIONPLANNER) {
                    printf("WARNNING! The compid is %u\n", recvMsgPtr->compid);
                }

                printf("In mission clear all with status: %d\n", (int)m_status);

                switch(m_status) {
                    case idle:
                    case uploading:
                    case downloading:
                    case loaded:
                    case error:
                        m_status = idle;
                        break;
                    case executing:
                    case paused:
                        return;
                }

                mavlink_mission_ack_t ackMsg;
                ackMsg.target_system = recvMsgPtr->sysid;
                ackMsg.target_component = recvMsgPtr->compid;
                ackMsg.type = MAV_MISSION_ACCEPTED;

                mavlink_message_t sendMsg;
                mavlink_msg_mission_ack_encode(m_hdlr->getSysid(), 
                        MAV_COMP_ID_MISSIONPLANNER, &sendMsg, &ackMsg);
                m_hdlr->sendEncodedMsg( getMasterGcsIdx(), &sendMsg ); 
                sendMsgToMaster(sendMsg);

                m_hp.clear();

                if(NULL != m_missionClearAllRsp)
                    m_missionClearAllRsp();

            }


            void reactToMissionSetCurrent(uint16_t gcsIdx, 
                    const mavlink_message_t* recvMsgPtr) {

                if(recvMsgPtr->compid != MAV_COMP_ID_MISSIONPLANNER) {
                    printf("WARNNING! The compid is %u\n", recvMsgPtr->compid);
                }

                printf("In mission set current with status: %d\n", (int)m_status);

                switch(m_status) {
                    case idle:
                    case uploading:
                    case downloading:
                    case error:
                        return;
                    case loaded:
                    case paused:
                        m_status = executing;
                        break;
                    case executing:
                        break;
                }

                mavlink_mission_set_current_t setCurrMsg;
                mavlink_msg_mission_set_current_decode(recvMsgPtr, &setCurrMsg);
                if(0 == setCurrMsg.seq) {

                    if(NULL != m_targetRsp) {
                        m_targetRsp( m_hp.getHotpoint(), 7, m_hp.getCmd() );
                    } else {
                        printf("Warning! No target responser is set!\n");
                    }

printf("Finish running target rsp\n");

                    mavlink_message_t sendMsg;
                    mavlink_msg_mission_current_pack( m_hdlr->getSysid(), 
                            MAV_COMP_ID_MISSIONPLANNER, &sendMsg, 0 );
                    sendMsgToMaster(sendMsg);

                    m_status = loaded;

                } else {
                    m_status = idle;
                    printf("Invalid sequence number %u", setCurrMsg.seq);
                }

                if(NULL != m_missionSetCurrentRsp)
                    m_missionSetCurrentRsp(setCurrMsg.seq);

            }


            void setMissionRequestListRsp(void (*func)()) {
                m_missionRequestListRsp = func;
            }


            void setMissionRequestRsp(void (*func)(uint16_t)) {
                m_missionRequestRsp = func;
            }


            void setMissionAckRsp(void (*func)()) {
                m_missionAckRsp = func;
            }


            void setMissionCountRsp(void (*func)(uint16_t)) {
                m_missionCountRsp = func;
            }


            void setMissionItemRsp(void (*func)(uint16_t)) {
                m_missionItemRsp = func;
            }


            void setMissionClearAllRsp(void (*func)()) {
                m_missionClearAllRsp = func;
            }


            void setMissionSetCurrentRsp(void (*func)(uint16_t)) {
                m_missionSetCurrentRsp = func;
            }


            void setTargetRsp(void (*func)(const float[], uint16_t, uint16_t)) {
                m_targetRsp = func;
            }



        private:
            MavHandler* m_hdlr;
            HotpointData m_hp;

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
            void (*m_targetRsp)(const float[], uint16_t, uint16_t);


    };

}


#endif
