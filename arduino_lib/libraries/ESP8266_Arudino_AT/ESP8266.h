/**
 * @file ESP8266.h
 * @brief The definition of class ESP8266. 
 * @author Wu Pengfei<pengfei.wu@itead.cc> 
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
#ifndef __ESP8266_H__
#define __ESP8266_H__

#include "Arduino.h"

//#define ESP8266_USE_SOFTWARE_SERIAL

#define ESP8266_SUPPORT_VERSION     "AT version:0.25 or 0.40 or 1.3.0.0"
#define ESP8266_SUPPORT_VERSION_025 "AT version:0.25"
#define ESP8266_SUPPORT_VERSION_040 "AT version:0.40"
#define ESP8266_SUPPORT_VERSION_130 "AT version:1.3.0.0"

#define ESP8266_NUM_CONNECTION 5

#ifdef ESP8266_USE_SOFTWARE_SERIAL
#include "SoftwareSerial.h"
#endif /* ESP8266_USE_SOFTWARE_SERIAL */

#define ESP8266_CONNECTION_OPEN     0x01
#define ESP8266_CONNECTION_CLOSE    0x02

#define ESP8266_CONNECTION_TCP      0x01
#define ESP8266_CONNECTION_UDP      0x02

#define ESP8266_CONNECTION_CLIENT   0x01
#define ESP8266_CONNECTION_SERVER   0x02

#define ESP8266_STATUS_GOTIP        0x02
#define ESP8266_STATUS_CONNECTED    0x03
#define ESP8266_STATUS_DISCONNECTED 0x04

#define ESP8266_WMODE_STATION       0x01
#define ESP8266_WMODE_SOFTAP        0x02
#define ESP8266_WMODE_AP_STATION    0x03

struct esp8266_connection_status
{
    uint8_t connection_type;
    uint8_t protocol_type;
};

#define ESP8266_RINGBUFFER_SIZE  1024

class ESP8266_RingBuffer
{
  public:
    ESP8266_RingBuffer(void);
    void init(void);
    void free(void) {len = head = tail = 0;};
    bool write(uint8_t c);
    uint8_t read(void);
    uint32_t copy(uint8_t *pdata, uint32_t size);
    uint8_t peek(void);
    bool available(void) {return (len > 0);};
    bool isFull(void) {return (len == ESP8266_RINGBUFFER_SIZE);};
    uint32_t length(void) {return len;};
    
  private:
    uint8_t *buffer;
    uint32_t head;
    uint32_t tail;
    uint32_t len;
};

/**
 * Provide an easy-to-use way to manipulate ESP8266. 
 */
class ESP8266 {
 public:

    /*
     * Begin.
     *
     * @param uart - an reference of SoftwareSerial object. 
     * @param baud - the buad rate to communicate with ESP8266(default:115200). 
     *
     * @retval 0 : Success
     * @retval 1 : Can not communicate ESP8266
     * @retval 2 : Firmware Version mismatch.
     *      
     * @warning parameter baud depends on the AT firmware. 115200 is an common value.
     */
#ifdef ESP8266_USE_SOFTWARE_SERIAL    
    int begin(SoftwareSerial &uart, uint32_t baud = 115200);
#else /* HardwareSerial */
    int begin(HardwareSerial &uart, uint32_t baud = 115200);
#endif /* ESP8266_USE_SOFTWARE_SERIAL */
    
    /** 
     * Verify ESP8266 whether live or not. 
     *
     * Actually, this method will send command "AT" to ESP8266 and waiting for "OK". 
     * 
     * @retval true - alive.
     * @retval false - dead.
     */
    bool kick(void);
    
    /**
     * Restart ESP8266 by "AT+RST". 
     *
     * This method will take 3 seconds or more. 
     *
     * @retval true - success.
     * @retval false - failure.
     */
    bool restart(void);
    
    /**
     * Get the version of AT Command Set. 
     * 
     * @return the string of version. 
     */
    String getVersion(void);
    
    /**
     * Set operation mode to staion. 
     * 
     * @retval true - success.
     * @retval false - failure.
     */
    bool setOprToStation(void);
    
    /**
     * Set operation mode to softap. 
     * 
     * @retval true - success.
     * @retval false - failure.
     */
    bool setOprToSoftAP(void);
    
    /**
     * Set operation mode to station + softap. 
     * 
     * @retval true - success.
     * @retval false - failure.
     */
    bool setOprToStationSoftAP(void);
    
    /**
     * Search available AP list and return it.
     * 
     * @return the list of available APs. 
     * @note This method will occupy a lot of memeory(hundreds of Bytes to a couple of KBytes). 
     *  Do not call this method unless you must and ensure that your board has enough memery left.
     */
    String getAPList(void);
    
    /**
     * Join in AP. 
     *
     * @param ssid - SSID of AP to join in. 
     * @param pwd - Password of AP to join in. 
     * @retval true - success.
     * @retval false - failure.
     * @note This method will take a couple of seconds. 
     */
    bool joinAP(String ssid, String pwd);
    
    /**
     * Leave AP joined before. 
     *
     * @retval true - success.
     * @retval false - failure.
     */
    bool leaveAP(void);
    
    /**
     * Set SoftAP parameters. 
     * 
     * @param ssid - SSID of SoftAP. 
     * @param pwd - PASSWORD of SoftAP. 
     * @param chl - the channel (1 - 13, default: 7). 
     * @param ecn - the way of encrypstion (0 - OPEN, 1 - WEP, 
     *  2 - WPA_PSK, 3 - WPA2_PSK, 4 - WPA_WPA2_PSK, default: 4). 
     * @note This method should not be called when station mode. 
     */
    bool setSoftAPParam(String ssid, String pwd, uint8_t chl = 7, uint8_t ecn = 4);
    
    /**
     * Get the IP list of devices connected to SoftAP. 
     * 
     * @return the list of IP.
     * @note This method should not be called when station mode. 
     */
    String getJoinedDeviceIP(void);
    
    /**
     * Get the current status of connection(UDP and TCP). 
     * 
     * @return the status. 
     */
    String getIPStatus(void);
    
    /**
     * Get the IP address of ESP8266. 
     *
     * @return the IP list. 
     */
    String getLocalIP(void);

    /**
     * Enable IP MUX(multiple connection mode). 
     *
     * In multiple connection mode, a couple of TCP and UDP communication can be builded. 
     * They can be distinguished by the identifier of TCP or UDP named mux_id. 
     * 
     * @retval true - success.
     * @retval false - failure.
     */
    bool enableMUX(void);
    
    /**
     * Disable IP MUX(single connection mode). 
     *
     * In single connection mode, only one TCP or UDP communication can be builded. 
     * 
     * @retval true - success.
     * @retval false - failure.
     */
    bool disableMUX(void);
    
    
    /**
     * Create TCP connection in single mode. 
     * 
     * @param addr - the IP or domain name of the target host. 
     * @param port - the port number of the target host. 
     * @retval true - success.
     * @retval false - failure.
     */
    bool createTCP(String addr, uint32_t port);
    
    /**
     * Release TCP connection in single mode. 
     * 
     * @retval true - success.
     * @retval false - failure.
     */
    bool releaseTCP(void);
    
    /**
     * Register UDP port number in single mode.
     * 
     * @param addr - the IP or domain name of the target host. 
     * @param port - the port number of the target host. 
     * @retval true - success.
     * @retval false - failure.
     */
    bool registerUDP(String addr, uint32_t port);
    
    /**
     * Unregister UDP port number in single mode. 
     * 
     * @retval true - success.
     * @retval false - failure.
     */
    bool unregisterUDP(void);
  
    /**
     * Create TCP connection in multiple mode. 
     * 
     * @param mux_id - the identifier of this TCP(available value: 0 - 4). 
     * @param addr - the IP or domain name of the target host. 
     * @param port - the port number of the target host. 
     * @retval true - success.
     * @retval false - failure.
     */
    bool createTCP(uint8_t mux_id, String addr, uint32_t port);
    
    /**
     * Release TCP connection in multiple mode. 
     * 
     * @param mux_id - the identifier of this TCP(available value: 0 - 4). 
     * @retval true - success.
     * @retval false - failure.
     */
    bool releaseTCP(uint8_t mux_id);
    
    /**
     * Register UDP port number in multiple mode.
     * 
     * @param mux_id - the identifier of this TCP(available value: 0 - 4). 
     * @param addr - the IP or domain name of the target host. 
     * @param port - the port number of the target host. 
     * @retval true - success.
     * @retval false - failure.
     */
    bool registerUDP(uint8_t mux_id, String addr, uint32_t port);
    
    /**
     * Unregister UDP port number in multiple mode. 
     * 
     * @param mux_id - the identifier of this TCP(available value: 0 - 4). 
     * @retval true - success.
     * @retval false - failure.
     */
    bool unregisterUDP(uint8_t mux_id);


    /**
     * Set the timeout of TCP Server. 
     * 
     * @param timeout - the duration for timeout by second(0 ~ 28800, default:180). 
     * @retval true - success.
     * @retval false - failure.
     */
    bool setTCPServerTimeout(uint32_t timeout = 180);
    
    /**
     * Start TCP Server(Only in multiple mode). 
     * 
     * After started, user should call method: getIPStatus to know the status of TCP connections. 
     * The methods of receiving data can be called for user's any purpose. After communication, 
     * release the TCP connection is needed by calling method: releaseTCP with mux_id. 
     *
     * @param port - the port number to listen(default: 333).
     * @retval true - success.
     * @retval false - failure.
     *
     * @see String getIPStatus(void);
     * @see uint32_t recv(uint8_t *coming_mux_id, uint8_t *buffer, uint32_t len, uint32_t timeout);
     * @see bool releaseTCP(uint8_t mux_id);
     */
    bool startTCPServer(uint32_t port = 333);

    /**
     * Stop TCP Server(Only in multiple mode). 
     * 
     * @retval true - success.
     * @retval false - failure.
     */
    bool stopTCPServer(void);
    
    /**
     * Start Server(Only in multiple mode). 
     * 
     * @param port - the port number to listen(default: 333).
     * @retval true - success.
     * @retval false - failure.
     *
     * @see String getIPStatus(void);
     * @see uint32_t recv(uint8_t *coming_mux_id, uint8_t *buffer, uint32_t len, uint32_t timeout);
     */
    bool startServer(uint32_t port = 333);

    /**
     * Stop Server(Only in multiple mode). 
     * 
     * @retval true - success.
     * @retval false - failure.
     */
    bool stopServer(void);

    /**
     * Send data based on TCP or UDP builded already in single mode. 
     * 
     * @param buffer - the buffer of data to send. 
     * @param len - the length of data to send. 
     * @retval true - success.
     * @retval false - failure.
     */
    bool send(const uint8_t *buffer, uint32_t len);
            
    /**
     * Send data based on one of TCP or UDP builded already in multiple mode. 
     * 
     * @param mux_id - the identifier of this TCP(available value: 0 - 4). 
     * @param buffer - the buffer of data to send. 
     * @param len - the length of data to send. 
     * @retval true - success.
     * @retval false - failure.
     */
    bool send(uint8_t mux_id, const uint8_t *buffer, uint32_t len);

    /**
     * Send data based on TCP or UDP builded already in single mode. 
     * 
     * @param str - String to send. 
     * @retval true - success.
     * @retval false - failure.
     */
    bool send(String &str);
    
    /**
     * Send data based on one of TCP or UDP builded already in multiple mode. 
     * 
     * @param mux_id - the identifier of this TCP(available value: 0 - 4). 
     * @param str - String to send. 
     * @retval true - success.
     * @retval false - failure.
     */
    bool send(uint8_t mux_id, String &str);
    
    /**
     * Receive data from TCP or UDP builded already in single mode. 
     *
     * @param buffer - the buffer for storing data. 
     * @param buffer_size - the length of the buffer. 
     * @param timeout - the time waiting data. 
     * @return the length of data received actually. 
     */
    uint32_t recv(uint8_t *buffer, uint32_t buffer_size, uint32_t timeout = 1000);
    
    /**
     * Receive data from one of TCP or UDP builded already in multiple mode. 
     *
     * @param mux_id - the identifier of this TCP(available value: 0 - 4). 
     * @param buffer - the buffer for storing data. 
     * @param buffer_size - the length of the buffer. 
     * @param timeout - the time waiting data. 
     * @return the length of data received actually. 
     */
    uint32_t recv(uint8_t mux_id, uint8_t *buffer, uint32_t buffer_size, uint32_t timeout = 1000);

    /**
     * Receive data from all of TCP or UDP builded already in multiple mode. 
     *
     * After return, coming_mux_id store the id of TCP or UDP from which data coming. 
     * User should read the value of coming_mux_id and decide what next to do. 
     * 
     * @param coming_mux_id - the identifier of TCP or UDP. 
     * @param buffer - the buffer for storing data. 
     * @param buffer_size - the length of the buffer. 
     * @param timeout - the time waiting data. 
     * @return the length of data received actually. 
     */
    uint32_t recv(uint8_t *coming_mux_id, uint8_t *buffer, uint32_t buffer_size, uint32_t timeout = 1000);
    
    /**
     * Check data is available in specific connection.
     * 
     * @param mux_id - the identifier of this TCP(available value: 0 - 4).      
     * @return true - data is available.
     * @return false - data is not available.
     */
    bool isDataAvailable(uint8_t mux_id);

    /**
     * Check data is available in any connection.
     *
     * @return true - data is available.
     * @return false - data is not available.
     */
    bool isDataAvailable(void);

    /**
     * Check connection is opned in single mode. 
     *
     * @return true  - opend.
     * @return false - closed.
     */
    bool isConnected(void);

    /**
     * Check specific connection is opned in multiple mode. 
     *
     * @return true  - opend.
     * @return false - closed.
     */
    bool isConnected(uint8_t mux_id);

    /**
     * Get connection link status
     *
     * @return true  - success.
     * @return false - error.
     */    
    bool getMuxCStatus(uint8_t *mux_id_ptn);

    friend class ESP8266Client;
    
 private:

    /* 
     * Empty the buffer or UART RX.
     */
    void rx_empty(void);

    /* 
     * Read all data in UART RX and store to Rceive Buffer .
     */
    void rx_update(void);    
     
    /* 
     * Recvive data from uart. Return all received data if one of target1, target2 and target3 found or timeout. 
     */
    int recvString(String target1, String target2, String target3, uint32_t timeout = 1000);
    
    /* 
     * Recvive data from uart and search first target. Return true if target found, false for timeout.
     */
    bool recvFind(String target, uint32_t timeout = 1000);
    
    /* 
     * Recvive data from uart and search first target and cut out the substring between begin and end(excluding begin and end self). a
     * Return true if target found, false for timeout.
     */
    bool recvFindAndFilter(String target, String begin, String end, String &data, uint32_t timeout = 1000);
    
    /*
     * Receive a package from uart. 
     *
     * @param buffer - the buffer storing data. 
     * @param buffer_size - guess what!
     * @param data_len - the length of data actually received(maybe more than buffer_size, the remained data will be abandoned).
     * @param timeout - the duration waitting data comming.
     * @param coming_mux_id - in single connection mode, should be NULL and not NULL in multiple. 
     */
    uint32_t recvPkg(uint8_t *buffer, uint32_t buffer_size, uint32_t timeout, uint8_t mux_id, uint8_t *coming_mux_id);
    
    bool eAT(void);
    bool eATRST(void);
    bool eATGMR(String &version);
    
    bool qATCWMODE_CUR(uint8_t *mode);
    bool sATCWMODE_CUR(uint8_t mode);
    bool sATCWJAP_CUR(String ssid, String pwd);
    bool eATCWLAP(String &list);
    bool eATCWQAP(void);
    bool sATCWSAP_CUR(String ssid, String pwd, uint8_t chl, uint8_t ecn);
    bool eATCWLIF(String &list);
    
    bool eATCIPSTATUS(String &list);
    bool sATCIPSTARTSingle(String type, String addr, uint32_t port);
    bool sATCIPSTARTMultiple(uint8_t mux_id, String type, String addr, uint32_t port);
    bool sATCIPSENDSingle(const uint8_t *buffer, uint32_t len);
    bool sATCIPSENDMultiple(uint8_t mux_id, const uint8_t *buffer, uint32_t len);
    bool sATCIPSENDSingle(String &str);
    bool sATCIPSENDMultiple(uint8_t mux_id, String &str);    
    bool sATCIPCLOSEMulitple(uint8_t mux_id);
    bool eATCIPCLOSESingle(void);
    bool eATCIFSR(String &list);
    bool sATCIPMUX(uint8_t mode);
    bool sATCIPSERVER(uint8_t mode, uint32_t port = 333);
    bool sATCIPSTO(uint32_t timeout);

    void initialize_status(void);
    
    uint32_t recvIPD(uint32_t timeout, uint8_t *buffer = NULL, uint32_t buffer_size = 0,
                     uint8_t mux_id = 0, uint8_t *coming_mux_id = NULL);

    void recvAsyncdata(uint32_t timeout = 0);

    /**
     * Restart ESP8266 by "AT+RST". 
     *
     * This method will take 3 seconds or more. 
     *
     * @retval true - success.
     * @retval false - failure.
     */
    bool _restart(void);    
    
#ifdef ESP8266_USE_SOFTWARE_SERIAL
    SoftwareSerial *m_puart; /* The UART to communicate with ESP8266 */
#else
    HardwareSerial *m_puart; /* The UART to communicate with ESP8266 */
#endif
    uint32_t m_baud;

    uint8_t wifi_status;
    
    bool    mux_mode;
    
    uint8_t connection_bitmap;
    
    esp8266_connection_status connection_status[ESP8266_NUM_CONNECTION];

    String rx_cmd;

  protected:
    /**
     *  Rceive Buffer for each link.
     */ 
    ESP8266_RingBuffer rx_buffer[ESP8266_NUM_CONNECTION];    
};

extern ESP8266 WiFi;

#endif /* #ifndef __ESP8266_H__ */

