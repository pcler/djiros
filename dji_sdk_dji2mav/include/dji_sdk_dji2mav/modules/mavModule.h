/*****************************************************************************
 * @Brief     Base class of dji2mav module. ROS-free and mav-depended
 * @Version   0.2.2
 * @Author    Chris Liu
 * @Created   2015/12/06
 * @Modified  2015/12/11
 *****************************************************************************/

#ifndef _MAV2DJI_MAVMODULE_H_
#define _MAV2DJI_MAVMODULE_H_


#include "moduleBuf.h"

#include <mavlink/common/mavlink.h>
#include <iostream>
#include <stdio.h>
#include <new>

namespace dji2mav{

    class MavModule {

        public:
            MavModule(uint16_t bufSize) {
                m_hdlr = MavHandler::getInstance();
                m_masterGcsIdx = -1;

                try {
                    m_senderRecord = new int[m_hdlr->getMngListSize()];
                    memset( m_senderRecord, 0, 
                            m_hdlr->getMngListSize() * sizeof(int) );
                    m_moduleBuf = new ModuleBuf(bufSize);
                } catch(std::bad_alloc& m) {
                    std::cerr << "Failed to alloc memory for mavModule: " 
                            << "at line: " << __LINE__ << ", func: " 
                            << __func__ << ", file: " << __FILE__ 
                            << std::endl;
                    perror( m.what() );
                    exit(EXIT_FAILURE);
                }

            }


            virtual ~MavModule() {
                if(NULL != m_senderRecord) {
                    delete []m_senderRecord;
                    m_senderRecord = NULL;
                }
                m_hdlr = NULL;
            }


            /**
             * @brief  Get the index of master GCS
             * @return The index of master GCS
             */
            inline int getMasterGcsIdx() {
                return m_masterGcsIdx;
            }


            /**
             * @brief  Get the sender index of master GCS
             * @return The sender index of master GCS or -2 for no master
             */
            int getMasterGcsSenderIdx() {
                if(-1 == m_masterGcsIdx)
                    return -2;
                else
                    return m_senderRecord[m_masterGcsIdx];
            }


            /**
             * @brief  Set the master GCS with its index
             * @param  gcsIdx    : The index of GCS that is to be set
             * @return True if succeed or false if fail
             */
            bool setMasterGcsIdx(uint16_t gcsIdx) {
                if( !activateSender(gcsIdx) ) {
                    printf("Fail to set master GCS to #%u\n", gcsIdx);
                    return false;
                }
                if(-1 != m_masterGcsIdx) {
                    //TODO: m_hdlr->unregister(m_senderRecord[m_masterGcsIdx]);
                    m_senderRecord[m_masterGcsIdx] = -1;
                }
                //at last, turn on master mode
                m_masterGcsIdx = gcsIdx;
                return true;
            }


            /**
             * @brief  Get the senderRecord array address
             * @return The address of senderRecord
             */
            inline const int* getSenderRecord() {
                return m_senderRecord;
            }


            /**
             * @brief  Get sender index for specific GCS
             * @param  gcsIdx : The index of GCS
             * @return The sender index of the GCS or -2 for invalid input
             */
            int getGcsSenderIdx(uint16_t gcsIdx) {
                if( !m_hdlr->isValidMngIdx(gcsIdx) )
                    return -2;
                return m_senderRecord[gcsIdx];
            }


            /**
             * @brief  Employ a sender for the specific GCS
             * @param  gcsIdx : The index of GCS
             * @return True if succeed or false if fail
             */
            bool employGcsSender(uint16_t gcsIdx) {
                if( !activateSender(gcsIdx) ) {
                    printf("Fail to employ a sender for GCS #%u\n", gcsIdx);
                    return false;
                }
                //turn off master mode
                m_masterGcsIdx = -1;
                return true;
            }


            /**
             * @brief  Activate sender of the specific GCS
             * @param  gcsIdx : The index of GCS
             * @return True if succeed or false if fail
             */
            bool activateSender(uint16_t gcsIdx) {
                if( !m_hdlr->isValidMngIdx(gcsIdx) ) {
                    printf("Invalid GCS index: %u\n", gcsIdx);
                    return false;
                }
                int sender = m_senderRecord[gcsIdx];
                if( !m_hdlr->isValidIdx(gcsIdx, sender) ) {
                    sender = m_hdlr->registerSender(gcsIdx);
                    if(-1 == sender) {
                        printf("Fail to register sender for GCS #%u\n", 
                                gcsIdx);
                        return false;
                    }
                }
                m_senderRecord[gcsIdx] = sender;
                return true;
            }


            /**
             * @brief  Send message to specific GCS
             * @param  gcsIdx : The index of specific GCS
             * @param  msg    : The reference of encoded mavlink message
             * @return True if succeed or false if fail
             */
            bool sendMsgToGcs(uint16_t gcsIdx, mavlink_message_t& msg) {
                //TODO: move valid check to handler
                bool ret = m_hdlr->sendEncodedMsg(gcsIdx, m_senderRecord[gcsIdx], &msg);
                if(false == ret)
                    printf("Cannot send message to GCS #%u\n", gcsIdx);
                return ret;
            }


            /**
             * @brief  Send message to master GCS
             * @param  msg : The reference of encoded mavlink message
             * @return True if succeed or false if fail
             */
            inline bool sendMsgToMaster(mavlink_message_t& msg) {
                return sendMsgToGcs(m_masterGcsIdx, msg);
            }


            /**
             * @brief  Send message to all activated GCS
             * @param  msg : The reference of encoded mavlink message
             * @return True if succeed or false if fail
             */
            bool sendMsgToAll(mavlink_message_t& msg) {
                bool ret = true;
                if(-1 != m_masterGcsIdx) {
                    return sendMsgToMaster(msg);
                } else {
                    bool ret = true;
                    for(uint16_t i = 0; i < m_hdlr->getMngListSize(); ++i) {
                        if( -1 != m_senderRecord[i] && !(ret &= 
                                sendMsgToGcs(m_senderRecord[i], msg)) ) {
                            printf("Fail to send message to GCS #%u\n", i);
                        }
                    }
                    if(!ret)
                        printf("Fail to send the message to some GCS\n");
                    return ret;
                }
            }


            /**
             * @brief  Push mavlink message to the buffer
             * @param  srcMsg : The source of message
             * @return True if succeed or false if fail
             */
            inline bool pushMsg(mavlink_message_t& srcMsg) {

                return m_moduleBuf->writeBuf( (uint8_t*)&srcMsg, 
                        sizeof(mavlink_message_t) );

            }


            /**
             * @brief  Pull mavlink message from the buffer
             * @param  destMsg : The destination of message
             * @return True if succeed or false if fail
             */
            inline bool pullMsg(mavlink_message_t& destMsg) {

                return m_moduleBuf->readBuf( (uint8_t*)&destMsg, 
                        sizeof(mavlink_message_t) );

            }


            /**
             * @brief  Run a thread for the module
             * @return True if succeed to create the thread or false if fail
             */
            bool run() {
                int ret = pthread_create(&m_tid, NULL, thread_call, (
                        void*)this);
                if(0 != ret) {
                    printf("Fail to create thread for the module.\n");
                    return false;
                }
                return true;
            }


            /**
             * @brief A thread calling function for the module
             * @param param : The pointer to the module object
             */
            static void* thread_call(void* param) {
                ( (MavModule*)param )->deliverMsg();

            }


            /**
             * @brief The message delivery function for the module. Pure virtual
             */
            virtual void deliverMsg() = 0;


        private:
            int* m_senderRecord;
            int m_masterGcsIdx;
            ModuleBuf* m_moduleBuf;

            MavHandler* m_hdlr;

            pthread_t m_tid;

    };

} //namespace dji2mav


#endif
