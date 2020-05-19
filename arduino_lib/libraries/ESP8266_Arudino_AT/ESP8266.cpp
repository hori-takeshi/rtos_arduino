/**
 * @file ESP8266.cpp
 * @brief The implementation of class ESP8266. 
 * @author Wu Pengfei<pengfei.wu@itead.cc>, Shinya Honda<honda@ertl.jp> 
 * @date 2015.02
 * 
 * @par Copyright:
 * Copyright (c) 2015 ITEAD Intelligent Systems Co., Ltd. \n\n
 * Copyright (C) 2015 Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN \n\n
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version. \n\n
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "ESP8266.h"

#define LOG_OUTPUT_DEBUG            (1)
#define LOG_OUTPUT_DEBUG_PREFIX     (1)

#define MUXID_ANY 255
#define MUXID_SINGLE 0

#ifdef TOPPERS_WITH_ARDUINO
#include "r2ca.h"
#define WAIT_TIMEOUT delay(1);
#define ENTER_CRITICAL wai_sem(ESP8266_SEM);
#define LEAVE_CRITICAL sig_sem(ESP8266_SEM);
#else /* !TOPPERS_WITH_ARDUINO */
#define WAIT_TIMEOUT
#define ENTER_CRITICAL
#define LEAVE_CRITICAL
#endif /* TOPPERS_WITH_ARDUINO */

#ifdef TOPPERS_WITH_ARDUINO
#define WAIT_TIMEOUT delay(1);
#else /* !TOPPERS_WITH_ARDUINO */
#define WAIT_TIMEOUT
#endif /* TOPPERS_WITH_ARDUINO */

#define logDebug(arg)\
    do {\
        if (LOG_OUTPUT_DEBUG)\
        {\
            if (LOG_OUTPUT_DEBUG_PREFIX)\
            {\
                Serial.print("[LOG Debug: ");\
                Serial.print((const char*)__FILE__);\
                Serial.print(",");\
                Serial.print((unsigned int)__LINE__);\
                Serial.print(",");\
                Serial.print((const char*)__FUNCTION__);\
                Serial.print("] ");\
            }\
            Serial.print(arg);\
        }\
    } while(0)

ESP8266_RingBuffer::ESP8266_RingBuffer(void)
{
    free(); 
    buffer = NULL;
}

void ESP8266_RingBuffer::init(void)
{
    free();
    if(buffer!=NULL) {
        ::free(buffer);
        buffer = NULL;
    }
}

bool ESP8266_RingBuffer::write(uint8_t c)
{
    if (len == ESP8266_RINGBUFFER_SIZE) {return false;}  

    if (buffer == NULL) {
        buffer = (uint8_t*)malloc(ESP8266_RINGBUFFER_SIZE);
        if(buffer == NULL) {return false;}
    }

    buffer[tail++] = c;
    len++;

    if (tail == ESP8266_RINGBUFFER_SIZE) {
        tail = 0;
    }
    return true;
}

uint8_t ESP8266_RingBuffer::read(void)
{
    uint8_t c;
    if (len == 0) {return 0;}
    c = buffer[head++];
    len--;
    if (head == ESP8266_RINGBUFFER_SIZE) {
        head = 0;
    }
    return c;
}

uint32_t ESP8266_RingBuffer::copy(uint8_t *pdata, uint32_t size)
{
    uint32_t ret;
    uint32_t pdata_index = 0;

    /* copy size */
    ret = (len < size)? len : size;
    
    while((pdata_index < ret)) {
        pdata[pdata_index++] = buffer[head++];
        if (head == ESP8266_RINGBUFFER_SIZE) {
            head = 0;
        }
    }

    len -= ret;

    return ret;
}

uint8_t ESP8266_RingBuffer::peek(void)
{
    if (len == 0) {return 0;}    
    return buffer[head];
}

void ESP8266::initialize_status(void)
{
    wifi_status = ESP8266_STATUS_DISCONNECTED;
    connection_bitmap = 0;
    mux_mode = false;
    rx_cmd = "";
    for(int i = 0; i < ESP8266_NUM_CONNECTION; i++){
        rx_buffer[i].init();
    }
}

#ifdef ESP8266_USE_SOFTWARE_SERIAL
int ESP8266::begin(SoftwareSerial &uart, uint32_t baud)
#else /* !ESP8266_USE_SOFTWARE_SERIAL */
int ESP8266::begin(HardwareSerial &uart, uint32_t baud)
#endif /* ESP8266_USE_SOFTWARE_SERIAL */
{
    String version;
    int ret = 0;
    
    ENTER_CRITICAL;
    m_puart = &uart;
    m_baud = baud;
    m_puart->begin(baud);
    initialize_status();
    rx_empty();
    
    if (eAT()) {
        eATGMR(version);
        if(version.indexOf(ESP8266_SUPPORT_VERSION_025) == -1) {
            if(version.indexOf(ESP8266_SUPPORT_VERSION_040) == -1) {
                if(version.indexOf(ESP8266_SUPPORT_VERSION_130) == -1) {
                    ret = 2;
                }
            }
        }
    }
    else {        
        ret = 1;
    }
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::kick(void)
{
    bool ret;
    
    ENTER_CRITICAL;
    ret = eAT();
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::_restart(void)
{
    unsigned long start;
    bool ret = false;
    
    initialize_status();
    if (eATRST()) {
        m_puart->end();
        delay(2000);
        m_puart->begin(m_baud);
        start = millis();
        while (millis() - start < 3000) {
            if (eAT()) {
                delay(1500); /* Waiting for stable */
                ret = true;
            }
            delay(100);
        }
    }
    
    return ret;
}

bool ESP8266::restart(void)
{
    bool ret;
    
    ENTER_CRITICAL;
    ret = _restart();
    LEAVE_CRITICAL;
    
    return ret;
}

String ESP8266::getVersion(void)
{
    String version;
    
    ENTER_CRITICAL;
    eATGMR(version);
    LEAVE_CRITICAL;
    
    return version;
}

bool ESP8266::setOprToStation(void)
{
    uint8_t mode;
    bool ret = false;

    ENTER_CRITICAL;
    if (qATCWMODE_CUR(&mode)) {
        if (mode == ESP8266_WMODE_STATION) {
            ret = true;
        } else {
            if (_restart() && sATCWMODE_CUR(ESP8266_WMODE_STATION)) {
                ret = true;
            }
        }
    }
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::setOprToSoftAP(void)
{
    uint8_t mode;
    bool ret = false;

    ENTER_CRITICAL;
    if (qATCWMODE_CUR(&mode)) {
        if (mode == ESP8266_WMODE_SOFTAP) {
            ret = true;
        } else {
            if (_restart() && sATCWMODE_CUR(ESP8266_WMODE_SOFTAP)) {
                ret = true;
            }
        }
    }
    LEAVE_CRITICAL;

    return ret;
}

bool ESP8266::setOprToStationSoftAP(void)
{
    uint8_t mode;
    bool ret = false;

    ENTER_CRITICAL;
    if (qATCWMODE_CUR(&mode)) {
        if (mode == ESP8266_WMODE_AP_STATION) {
            ret = true;
        } else {
            if (_restart() && sATCWMODE_CUR(ESP8266_WMODE_AP_STATION)) {
                ret = true;
            }
        }
    }
    LEAVE_CRITICAL;

    return ret;    
}

String ESP8266::getAPList(void)
{
    String list;
    
    ENTER_CRITICAL;
    eATCWLAP(list);
    LEAVE_CRITICAL;
    
    return list;
}

bool ESP8266::joinAP(String ssid, String pwd)
{
    bool ret;

    ENTER_CRITICAL;
    ret = sATCWJAP_CUR(ssid, pwd);
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::leaveAP(void)
{
    bool ret;

    ENTER_CRITICAL;
    ret = eATCWQAP();
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::setSoftAPParam(String ssid, String pwd, uint8_t chl, uint8_t ecn)
{
    bool ret;

    ENTER_CRITICAL;
    ret = sATCWSAP_CUR(ssid, pwd, chl, ecn);
    LEAVE_CRITICAL;
    
    return ret;
}

String ESP8266::getJoinedDeviceIP(void)
{
    String list;

    ENTER_CRITICAL;
    eATCWLIF(list);
    LEAVE_CRITICAL;
    
    return list;
}

String ESP8266::getIPStatus(void)
{
    String list;

    ENTER_CRITICAL;
    eATCIPSTATUS(list);
    LEAVE_CRITICAL;
    
    return list;
}

String ESP8266::getLocalIP(void)
{
    String list = "";
    int32_t index_start = -1;
    int32_t index_end = -1;

    ENTER_CRITICAL;
    eATCIFSR(list);
    index_start = list.indexOf('\"');
    index_end   = list.indexOf('\"', list.indexOf('\"')+1);
    if ((index_start !=  -1) && (index_end != -1)) {
        list = list.substring(index_start+1, index_end);
    }
    LEAVE_CRITICAL;
    
    return list;
}

bool ESP8266::enableMUX(void)
{
    bool ret = false;

    ENTER_CRITICAL;
    if (sATCIPMUX(1)) {
        mux_mode = true;
        ret = true;
    }
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::disableMUX(void)
{
    bool ret = false;

    ENTER_CRITICAL;
    if (sATCIPMUX(0)){
        mux_mode = false;
        ret = true;
    }
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::createTCP(String addr, uint32_t port)
{
    bool ret = false;
    if (mux_mode) return false;

    ENTER_CRITICAL;
    if((ret = sATCIPSTARTSingle("TCP", addr, port))) {
        connection_bitmap = 1;
    }
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::releaseTCP(void)
{
    bool ret = false;    
    if (mux_mode) return false;

    ENTER_CRITICAL;
    if((ret = eATCIPCLOSESingle())) {
        connection_bitmap = 0;
        rx_buffer[MUXID_SINGLE].free();
    }
    LEAVE_CRITICAL;
    
    return ret;    
}

bool ESP8266::registerUDP(String addr, uint32_t port)
{
    bool ret = false;
    if (mux_mode) return false;

    ENTER_CRITICAL;
    if((ret = sATCIPSTARTSingle("UDP", addr, port))) {
        connection_bitmap = 1;
    }
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::unregisterUDP(void)
{
    bool ret = false;
    if (mux_mode) return false;

    ENTER_CRITICAL;
    if((ret = eATCIPCLOSESingle())) {
        connection_bitmap = 0;
        rx_buffer[MUXID_SINGLE].free();
    }
    LEAVE_CRITICAL;
    
    return ret;    
}

bool ESP8266::createTCP(uint8_t mux_id, String addr, uint32_t port)
{
    bool ret = false;
    if (!(mux_id < ESP8266_NUM_CONNECTION)) return false;
    if (!mux_mode) return false;

    ENTER_CRITICAL;    
    if((ret = sATCIPSTARTMultiple(mux_id, "TCP", addr, port))) {
        connection_bitmap |= 1 << mux_id;
    }
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::releaseTCP(uint8_t mux_id)
{
    bool ret;
    if (!(mux_id < ESP8266_NUM_CONNECTION)) return false;
    if (!mux_mode) return false;

    ENTER_CRITICAL;
    if ((ret = sATCIPCLOSEMulitple(mux_id))) {
        connection_bitmap &= ~(1 << mux_id);
        rx_buffer[mux_id].free();
    }
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::registerUDP(uint8_t mux_id, String addr, uint32_t port)
{
    bool ret;
    if (!(mux_id < ESP8266_NUM_CONNECTION)) return false;
    if (!mux_mode) return false;

    ENTER_CRITICAL;
    if ((ret = sATCIPSTARTMultiple(mux_id, "UDP", addr, port))) {
        connection_bitmap |= 1 << mux_id;
    }
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::unregisterUDP(uint8_t mux_id)
{
    bool ret;    
    if (!(mux_id < ESP8266_NUM_CONNECTION)) return false;
    if (!mux_mode) return false;

    ENTER_CRITICAL;
    if ((ret = sATCIPCLOSEMulitple(mux_id))) {
        connection_bitmap &= ~(1 << mux_id);
        rx_buffer[mux_id].free();
    }
    LEAVE_CRITICAL;
    
    return ret;    
}

bool ESP8266::setTCPServerTimeout(uint32_t timeout)
{
    bool ret;
    if (!mux_mode) return false;

    ENTER_CRITICAL;
    ret = sATCIPSTO(timeout);
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::startTCPServer(uint32_t port)
{
    bool ret = false;
    if (!mux_mode) return false;

    ENTER_CRITICAL;
    ret = sATCIPSERVER(1, port);
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::stopTCPServer(void)
{
    bool ret = false;
    
    ENTER_CRITICAL;
    ret = sATCIPSERVER(0);
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::startServer(uint32_t port)
{
    bool ret = false;    
    if (!mux_mode) return false;
    
    ret = startTCPServer(port);
    
    return ret;
}

bool ESP8266::stopServer(void)
{
    bool ret;

    ret = stopTCPServer();
    
    return ret;
}

bool ESP8266::send(const uint8_t *buffer, uint32_t len)
{
    bool ret;
    if (mux_mode) return false;

    ENTER_CRITICAL;
    ret = sATCIPSENDSingle(buffer, len);
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::send(uint8_t mux_id, const uint8_t *buffer, uint32_t len)
{
    bool ret;
    if (!mux_mode) return false;

    ENTER_CRITICAL;
    ret = sATCIPSENDMultiple(mux_id, buffer, len);
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::send(String &str)
{
    bool ret;
    if (mux_mode) return 0;

    ENTER_CRITICAL;
    ret = sATCIPSENDSingle(str);
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::send(uint8_t mux_id, String &str)
{
    bool ret;
    if (!mux_mode) return false;
    if (!(mux_id < ESP8266_NUM_CONNECTION)) return false;

    ENTER_CRITICAL;
    ret = sATCIPSENDMultiple(mux_id, str);
    LEAVE_CRITICAL;
    
    return ret;
}

uint32_t ESP8266::recv(uint8_t *buffer, uint32_t buffer_size, uint32_t timeout)
{
    uint32_t ret;
    if (mux_mode) return 0;

    ENTER_CRITICAL;
    ret = recvPkg(buffer, buffer_size, timeout, 0, NULL);
    LEAVE_CRITICAL;
    
    return ret;
}

uint32_t ESP8266::recv(uint8_t mux_id, uint8_t *buffer, uint32_t buffer_size, uint32_t timeout)
{
    uint32_t ret;
    if (!mux_mode) return false;
    if (!(mux_id < ESP8266_NUM_CONNECTION)) return false;

    ENTER_CRITICAL;
    ret = recvPkg(buffer, buffer_size, timeout, mux_id, NULL);
    LEAVE_CRITICAL;
    
    return ret;
}

uint32_t ESP8266::recv(uint8_t *coming_mux_id, uint8_t *buffer, uint32_t buffer_size, uint32_t timeout)
{
    uint32_t ret;
    if (!mux_mode) return false;

    ENTER_CRITICAL;
    ret = recvPkg(buffer, buffer_size, timeout, MUXID_ANY, coming_mux_id);
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::isDataAvailable(uint8_t mux_id)
{
    bool ret;
    if (!mux_mode) return false;
    if (!(mux_id < ESP8266_NUM_CONNECTION)) return false;

    ENTER_CRITICAL;
    if(rx_buffer[mux_id].length() > 0) ret = true;
    if (ret == false) {
        rx_update();
        if(rx_buffer[mux_id].length() > 0) ret = true;
    }    
    LEAVE_CRITICAL;
        
    return ret;
}

bool ESP8266::isDataAvailable(void)
{
    bool ret = false;

    ENTER_CRITICAL;
    if (mux_mode) {
        for(int i = 0; i < ESP8266_NUM_CONNECTION; i++){
            if(rx_buffer[i].length() > 0) ret = true;
        }
        if (ret == false) {
            rx_update();
            for(int i = 0; i < ESP8266_NUM_CONNECTION; i++){
                if(rx_buffer[i].length() > 0) ret = true;
            }
        }
    }
    else {
        /* single mode */
        if(rx_buffer[0].length() > 0) ret = true;
        if (ret == false) {
            rx_update();
            if(rx_buffer[0].length() > 0) ret = true;
        }
    }
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::isConnected(void)
{
    bool ret;
    if (mux_mode) return false;
    
    ENTER_CRITICAL;
    rx_update();
    ret = (connection_bitmap == 1);
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::isConnected(uint8_t mux_id)
{
    bool ret;
    if (!mux_mode) return false;    
    if (!(mux_id < ESP8266_NUM_CONNECTION)) return false;

    ENTER_CRITICAL;
    rx_update();
    ret = ((connection_bitmap & (1<<mux_id)) != 0);
    LEAVE_CRITICAL;
    
    return ret;
}

bool ESP8266::getMuxCStatus(uint8_t *mux_id_ptn)
{
    if (!mux_mode) return false;

    ENTER_CRITICAL;
    rx_update();
    *mux_id_ptn = connection_bitmap;
    LEAVE_CRITICAL;
    
    return true;
}

/*----------------------------------------------------------------------------*/
/* +IPD,<id>,<len>:<data> */
/* +IPD,<len>:<data> */

uint32_t ESP8266::recvPkg(uint8_t *buffer, uint32_t buffer_size, uint32_t timeout,
                          uint8_t mux_id, uint8_t *coming_mux_id)
{    
    char a;
    unsigned long start;
    uint32_t ret;
    uint32_t recv_mux_id = 0;
    
    /*
     * Try to recive data from rx_buffer
     */
    if (mux_id == MUXID_ANY) {
        for(int i = 0; i < ESP8266_NUM_CONNECTION; i++){
            if(rx_buffer[i].length() > 0) {
                recv_mux_id = i;
                *coming_mux_id = i;
                break;
            }
        }
    }
    else {
        recv_mux_id = mux_id;
    }
    
    if(rx_buffer[recv_mux_id].length() > 0) {
        return rx_buffer[recv_mux_id].copy(buffer, buffer_size);
    }

    /*
     * Try to recive data from uart
     */    
    start = millis();
    while (millis() - start < timeout) {
        while(m_puart->available() > 0) {            
            a = m_puart->read();
            rx_cmd += a;
            if (rx_cmd.indexOf("+IPD,") != -1) {
                ret = recvIPD(timeout - (millis() - start), buffer, buffer_size,
                              mux_id, coming_mux_id);
                rx_cmd = "";
                return ret;
            } else {
                recvAsyncdata();
            }
        }
        WAIT_TIMEOUT;
    }
    return 0;
}

/*
 * call after read  +IPD,
 * +IPD,<id>,<len>:<data>
 * +IPD,<len>:<data>
 */
uint32_t ESP8266::recvIPD(uint32_t timeout, uint8_t *buffer, uint32_t buffer_size,
                          uint8_t mux_id, uint8_t *coming_mux_id)
{
    String data;
    char a;
    unsigned long start;
    int32_t index_colon = -1; /* : */
    int32_t index_comma = -1; /* , */
    int32_t len = -1;
    int8_t id = -1;
    bool has_data = false;
    uint32_t ret;
    uint32_t i;
    ESP8266_RingBuffer *p_rx_buffer;

    start = millis();
    while (millis() - start < timeout) {        
        if(m_puart->available() > 0) {
            a = m_puart->read();
            data += a;
            if((index_colon = data.indexOf(':')) != -1) {
                if(((index_comma = data.indexOf(',')) != -1) && index_comma < index_colon) {
                    /* +IPD,id,len:data */
                    id = data.substring(0, index_comma).toInt();
                    if (id < 0 || id > (ESP8266_NUM_CONNECTION-1)) {
                        return 0;
                    }
                    len = data.substring(index_comma + 1, index_colon).toInt();
                    if (len <= 0) {
                        return 0;
                    }
                }
                else {
                    /* +IPD,len:data */
                    len = data.substring(0, index_colon).toInt();
                    id = 0;
                    if (len <= 0) {
                        return 0;
                    }
                }
                has_data = true;
                break;
            }
        }
        else {
            WAIT_TIMEOUT;
        }
    }
    
    if (has_data) {
        p_rx_buffer = (id == -1)? &rx_buffer[0] : &rx_buffer[id];
        if (buffer == NULL || ((mux_id != MUXID_ANY) && (id != mux_id))) {
            /* Store in rx_buffer */        
            start = millis();
            i = 0;
            while (millis() - start < 3000) {
                while((m_puart->available() > 0) && (i < (uint32_t)len)) {
                    a = m_puart->read();
                    p_rx_buffer->write(a);
                    i++;
                }
                if (i == (uint32_t)len) {
                    /* id != mux_id */
                    return 0;
                }
                WAIT_TIMEOUT;
            }
            return 0;
        }
        else {
            /* Store in buffer */
            i = 0;
            ret = ((uint32_t)len > buffer_size) ? buffer_size : len;
            start = millis();
            while ((millis() - start) < 3000) {
                while((m_puart->available() > 0) && (i < (uint32_t)len)) {                    
                    a = m_puart->read();
                    if(i < buffer_size) {
                        buffer[i] = a;
                    } else {
                        p_rx_buffer->write(a);
                    }
                    i++;
                }
                if (i == (uint32_t)len) {
                    if (mux_id == MUXID_ANY) {
                        *coming_mux_id = id;
                    }
                    return ret;
                }
                WAIT_TIMEOUT;
            }
        }
        return i;
    }
    
    return 0;
}

void ESP8266::rx_empty(void) 
{
    while(m_puart->available() > 0) {
        m_puart->read();
    }
}

void ESP8266::recvAsyncdata(uint32_t timeout)
{
    int index;
    int id;

    if (rx_cmd.indexOf("+IPD,") != -1) {
        recvIPD(timeout);
        rx_cmd = "";
    } else if (rx_cmd.indexOf("CLOSED") != -1) {
        if ((index = rx_cmd.indexOf(",CLOSED")) != -1) {
            /* <link ID>,CLOSED */
            id = rx_cmd.substring(index-1, index).toInt();
            connection_bitmap &= ~(1 << id);
        } else {
            connection_bitmap = 0;                
        }
        rx_cmd = "";
    } else if ((index = rx_cmd.indexOf(",CONNECT")) != -1) {
        /* <link ID>,CONNECT */
        id = rx_cmd.substring(index-1, index).toInt();
        connection_bitmap |= (1 << id);
        rx_cmd = "";
    } else if ((index = rx_cmd.indexOf("WIFI CONNECTED")) != -1) {
        wifi_status = ESP8266_STATUS_CONNECTED;
        rx_cmd = "";
    } else if ((index = rx_cmd.indexOf("WIFI GOT IP")) != -1) {
        wifi_status = ESP8266_STATUS_GOTIP;
        rx_cmd = "";        
    } else if ((index = rx_cmd.indexOf("WIFI DISCONNECT")) != -1) {
        wifi_status = ESP8266_STATUS_DISCONNECTED;
        rx_cmd = "";        
    }    
}

void ESP8266::rx_update(void)
{
    char a;
    
    while(m_puart->available() > 0) {
        a = m_puart->read();
        rx_cmd += a;
        recvAsyncdata(3000);
    }
}

int ESP8266::recvString(String target1, String target2, String target3, uint32_t timeout)
{
    char a;
    unsigned long start = millis();
    while (millis() - start < timeout) {
        while (m_puart->available() > 0) {
            a = m_puart->read();
            rx_cmd += a;
            if (rx_cmd.indexOf(target1) != -1) {
                return 1;
            } else if ((target2.length() != 0 ) && (rx_cmd.indexOf(target2) != -1)) {
                return 2;
            } else if ((target3.length() != 0 ) && (rx_cmd.indexOf(target3) != -1)) {
                return 3;
            } else {
                recvAsyncdata(timeout - (millis() - start));
            }
        }
        WAIT_TIMEOUT;
    }
    return -1;
}

bool ESP8266::recvFind(String target, uint32_t timeout)
{
    int ret;
    
    ret = recvString(target, "", "", timeout);
    if (ret != -1) rx_cmd = "";
    return (ret == 1);
}

bool ESP8266::recvFindAndFilter(String target, String begin, String end, String &data, uint32_t timeout)
{
    if (recvString(target, "", "", timeout) == 1) {
        int32_t index1 = rx_cmd.indexOf(begin);
        int32_t index2 = rx_cmd.indexOf(end);
        if (index1 != -1 && index2 != -1) {
            index1 += begin.length();
            data = rx_cmd.substring(index1, index2);
            rx_cmd = "";
            return true;
        }
    }
    data = "";
    return false;
}

bool ESP8266::eAT(void)
{
    rx_update();
    m_puart->println("AT");
    return recvFind("OK");
}

bool ESP8266::eATRST(void) 
{
    rx_update();
    m_puart->println("AT+RST");
    return recvFind("OK");
}

bool ESP8266::eATGMR(String &version)
{
    rx_update();
    m_puart->println("AT+GMR");
    return recvFindAndFilter("OK", "\r\r\n", "\r\nOK", version); 
}
/* checked */
bool ESP8266::qATCWMODE_CUR(uint8_t *mode) 
{
    String str_mode;
    bool ret;
    if (!mode) {
        return false;
    }
    rx_update();
    m_puart->println("AT+CWMODE_CUR?");
    ret = recvFindAndFilter("OK", "+CWMODE_CUR:", "\r\n\r\nOK", str_mode); 
    if (ret) {
        *mode = (uint8_t)str_mode.toInt();
        return true;
    } else {
        return false;
    }
}
/* checked */
bool ESP8266::sATCWMODE_CUR(uint8_t mode)
{
    int ret;

    rx_update();
    m_puart->print("AT+CWMODE_CUR=");
    m_puart->println(mode);

    ret = recvString("OK", "ERROR", "");
    if (ret != -1) rx_cmd = "";
    return (ret == 1);
}
/* checked */
bool ESP8266::sATCWJAP_CUR(String ssid, String pwd)
{
    int ret;
    
    rx_update();
    m_puart->print("AT+CWJAP_CUR=\"");
    m_puart->print(ssid);
    m_puart->print("\",\"");
    m_puart->print(pwd);
    m_puart->println("\"");
    
    ret = recvString("OK", "FAIL", "", 10000);
    if (ret != -1) rx_cmd = "";
    return (ret == 1);
}

bool ESP8266::eATCWLAP(String &list)
{
    rx_update();
    m_puart->println("AT+CWLAP");
    return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", list, 10000);
}

bool ESP8266::eATCWQAP(void)
{
    rx_update();
    m_puart->println("AT+CWQAP");
    return recvFind("OK");
}
/* checked */
bool ESP8266::sATCWSAP_CUR(String ssid, String pwd, uint8_t chl, uint8_t ecn)
{
    int ret;
    
    rx_update();
    m_puart->print("AT+CWSAP_CUR=\"");
    m_puart->print(ssid);
    m_puart->print("\",\"");
    m_puart->print(pwd);
    m_puart->print("\",");
    m_puart->print(chl);
    m_puart->print(",");
    m_puart->println(ecn);
    
    ret = recvString("OK", "ERROR", "", 5000);
    if (ret != -1) rx_cmd = "";
    return (ret == 1);
}

bool ESP8266::eATCWLIF(String &list)
{
    rx_update();
    m_puart->println("AT+CWLIF");
    return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", list);
}

bool ESP8266::eATCIPSTATUS(String &list)
{
    rx_update();
    m_puart->println("AT+CIPSTATUS");
    return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", list);
}

bool ESP8266::sATCIPSTARTSingle(String type, String addr, uint32_t port)
{
    int ret;

    rx_update();
    m_puart->print("AT+CIPSTART=\"");
    m_puart->print(type);
    m_puart->print("\",\"");
    m_puart->print(addr);
    m_puart->print("\",");
    m_puart->println(port);
    
    ret = recvString("OK", "ERROR", "ALREADY CONNECT", 10000);
    if (ret != -1) rx_cmd = "";
    return (ret == 1 || ret == 3);
}

bool ESP8266::sATCIPSTARTMultiple(uint8_t mux_id, String type, String addr, uint32_t port)
{
    int ret;

    rx_update();
    m_puart->print("AT+CIPSTART=");
    m_puart->print(mux_id);
    m_puart->print(",\"");
    m_puart->print(type);
    m_puart->print("\",\"");
    m_puart->print(addr);
    m_puart->print("\",");
    m_puart->println(port);
    
    ret = recvString("OK", "ERROR", "ALREADY CONNECT", 10000);
    if (ret != -1) rx_cmd = "";
    return ((ret == 1)||(ret == 3));
}

bool ESP8266::sATCIPSENDSingle(const uint8_t *buffer, uint32_t len)
{
    bool ret;
    rx_update();
    m_puart->print("AT+CIPSEND=");
    m_puart->println(len);
    if (recvFind(">", 5000)) {
        for (uint32_t i = 0; i < len; i++) {
            m_puart->write(buffer[i]);
        }
        ret = recvFind("SEND OK", 10000);
        return ret;
    }
    
    return false;
}

bool ESP8266::sATCIPSENDMultiple(uint8_t mux_id, const uint8_t *buffer, uint32_t len)
{
    rx_update();
    m_puart->print("AT+CIPSEND=");
    m_puart->print(mux_id);
    m_puart->print(",");
    m_puart->println(len);
    if (recvFind(">", 5000)) {
        for (uint32_t i = 0; i < len; i++) {
            m_puart->write(buffer[i]);
        }
        return recvFind("SEND OK", 10000);
    }
    return false;
}

bool ESP8266::sATCIPSENDSingle(String &str)
{
    rx_update();
    m_puart->print("AT+CIPSEND=");
    m_puart->println(str.length());
    if (recvFind(">", 5000)) {
        for (uint32_t i = 0; i < str.length(); i++) {
            m_puart->write(str.charAt(i));
        }
        return recvFind("SEND OK", 10000);
    }
    return false;
}

bool ESP8266::sATCIPSENDMultiple(uint8_t mux_id, String &str)
{
    rx_update();
    m_puart->print("AT+CIPSEND=");
    m_puart->print(mux_id);
    m_puart->print(",");
    m_puart->println(str.length());
    if (recvFind(">", 5000)) {
        for (uint32_t i = 0; i < str.length(); i++) {
            m_puart->write(str.charAt(i));
        }
        return recvFind("SEND OK", 10000);
    }
    return false;
}

bool ESP8266::sATCIPCLOSEMulitple(uint8_t mux_id)
{
    int ret;
    
    rx_update();
    m_puart->print("AT+CIPCLOSE=");
    m_puart->println(mux_id);
    
    ret = recvString("OK", "link is not", "", 5000);
    if (ret != -1) rx_cmd = "";
    return ((ret == 1) || (ret == 2));
}

bool ESP8266::eATCIPCLOSESingle(void)
{
    int ret;
    
    Serial.println(rx_cmd);
    rx_update();
    m_puart->println("AT+CIPCLOSE");
    ret = recvString("OK", "ERROR", "", 5000);
    if (ret != -1) rx_cmd = "";
    return ((ret == 1) || (ret == 2));
}

bool ESP8266::eATCIFSR(String &list)
{
    rx_update();
    m_puart->println("AT+CIFSR");
    return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", list);
}
/* checked */
bool ESP8266::sATCIPMUX(uint8_t mode)
{
    int ret;

    rx_update();
    m_puart->print("AT+CIPMUX=");
    m_puart->println(mode);
    
    ret = recvString("OK", "ERROR", "");
    if (ret != -1) rx_cmd = "";
    return (ret == 1);    
}

bool ESP8266::sATCIPSERVER(uint8_t mode, uint32_t port)
{
    int ret;
    
    if (mode) {
        rx_update();
        m_puart->print("AT+CIPSERVER=1,");
        m_puart->println(port);        
        ret = recvString("OK", "ERROR", "");
        if (ret != -1) rx_cmd = "";
        return (ret == 1);
    } else {
        rx_update();
        m_puart->println("AT+CIPSERVER=0");
        ret = recvString("OK", "ERROR", "");
        if (ret != -1) rx_cmd = "";
        return (ret == 1);        
    }
}
bool ESP8266::sATCIPSTO(uint32_t timeout)
{
    rx_update();
    m_puart->print("AT+CIPSTO=");
    m_puart->println(timeout);
    return recvFind("OK");
}

ESP8266 WiFi;
