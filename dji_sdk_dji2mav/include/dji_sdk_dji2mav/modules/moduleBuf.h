/*****************************************************************************
 * @Brief     Module buf manager of the mavlink module. ROS-free and mav-free
 * @Version   0.2.2
 * @Author    Chris Liu
 * @Created   2015/12/06
 * @Modified  2015/12/10
 *****************************************************************************/

#ifndef _MAV2DJI_MODULEBUF_H_
#define _MAV2DJI_MODULEBUF_H_


#include <iostream>
#include <stdio.h>
#include <string.h>
#include <new>
#include <mutex>

namespace dji2mav{

    class ModuleBuf {
        public:
            ModuleBuf(uint16_t bufSize) {
                m_bufSize = bufSize;
                m_head = 0;
                m_bufUsedAmount = 0;

                try {
                    m_buf = new uint8_t[m_bufSize];
                    memset(m_buf, 0, sizeof(uint8_t) * m_bufSize);
                } catch(std::bad_alloc& m) {
                    std::cerr << "Failed to alloc memory for moduleBuf: "
                            << "at line: " << __LINE__ << ", func: "
                            << __func__ << ", file: " << __FILE__
                            << std::endl;
                    perror( m.what() );
                    exit(EXIT_FAILURE);
                }

            }


            ~ModuleBuf() {
                if(NULL != m_buf) {
                    delete []m_buf;
                    m_buf = NULL;
                }
            }


            /**
             * @brief  Get the max size of buffer
             * @return The max size of buffer
             */
            inline uint16_t getBufSize() {
                return m_bufSize;
            }


            /**
             * @brief  Get the used amount of buffer
             * @return The used amount of buffer
             */
            inline uint16_t getBufUsedAmount() {
                return m_bufUsedAmount;
            }


            /**
             * @brief   Write data to the buffer. Multi-thread supported
             * @param   src : The pointer to the source memory
             * @param   len : The length that is going to write to the buffer
             * @return  True if succeed or false if fail
             */
            bool writeBuf(uint8_t *src, uint16_t len) {

                if( m_bufUsedAmount + len > m_bufSize ) {
                    return false;
                } else {
                    m_bufMutex.lock();
                    if(len + m_head + m_bufUsedAmount < m_bufSize) {
                        memcpy(m_buf + m_head + m_bufUsedAmount, src, len);
                    } else {
                        uint16_t bSize = m_bufSize - m_head - m_bufUsedAmount;
                        memcpy(m_buf + m_head + m_bufUsedAmount, src, bSize);
                        memcpy(m_buf, src + bSize, len - bSize);
                    }
                    m_bufUsedAmount += len;
                    m_bufMutex.unlock();
                    return true;
                }

            }


            /**
             * @brief  Read data from the buffer. Multi-thread supported
             * @param  dest : The pointer to the destination memory
             * @param  len  : The length that is going to read from the buffer
             * @return True if succeed or false if fail
             */
            bool readBuf(uint8_t *dest, uint16_t len) {

                if( len > m_bufUsedAmount ) {
                    return false;
                } else {
                    m_bufMutex.lock();
                    if(m_head + len < m_bufSize) {
                        memcpy(dest, m_buf + m_head, len);
                        m_head += len;
                    } else {
                        memcpy(dest, m_buf + m_head, m_bufSize - m_head);
                        memcpy(dest + m_bufSize - m_head, m_buf, 
                                len + m_head - m_bufSize);
                        m_head -= m_bufSize - len;
                    }
                    m_bufUsedAmount -= len;
                    m_bufMutex.unlock();
                    return true;
                }
            }


            void display() {
                printf("head: %u, used: %u, buf: ", m_head, m_bufUsedAmount);
                for(int i = 0; i < m_bufSize; ++i) {
                    printf("%02x", m_buf[i]);
                }
                printf("\n");
            }


        private:
            uint8_t *m_buf;
            uint16_t m_bufSize;
            uint16_t m_head;
            uint16_t m_bufUsedAmount;
            std::mutex m_bufMutex;
    };

}

#endif
