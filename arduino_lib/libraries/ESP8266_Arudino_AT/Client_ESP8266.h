/*
 Copyright (C) 2015 Embedded and Real-Time Systems Laboratory
              Graduate School of Information Science, Nagoya Univ., JAPAN
  
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef _CLIENT_ESP8366_H_
#define _CLIENT_ESP8366_H_

#include "ESP8266.h"
#include "IPAddress.h"
#include "Client.h"

class ESP8266Client: public Client {
  public:
    ESP8266Client(void) {
        mux_id = 0;
        mux = false; 
        pbuffer = &(WiFi.rx_buffer[0]);
    }

    ESP8266Client(uint8_t _mux_id) {
        mux_id = _mux_id;
        mux = true; 
        pbuffer = &(WiFi.rx_buffer[mux_id]);
    }    
    
    /*
     * @retval 0 : faild
     * @retval 1 : success
     */
    int connect(IPAddress ip, uint16_t port) {
        if (mux) {
            if (WiFi.isConnected(mux_id)) return 1;
            String ip_str;
            ip_str.concat(ip[0]); ip_str.concat(".");
            ip_str.concat(ip[1]); ip_str.concat(".");
            ip_str.concat(ip[2]); ip_str.concat(".");
            ip_str.concat(ip[3]);
            return (WiFi.createTCP(mux_id, ip_str, port))? 1 : 0;            
        }
        else {
            if (WiFi.isConnected()) return 1;
            String ip_str;
            ip_str.concat(ip[0]); ip_str.concat(".");
            ip_str.concat(ip[1]); ip_str.concat(".");
            ip_str.concat(ip[2]); ip_str.concat(".");
            ip_str.concat(ip[3]);
            return (WiFi.createTCP(ip_str, port))? 1 : 0;
        }
    };
    
    int connect(const char *host, uint16_t port) {
        if (mux) {
            if (WiFi.isConnected(mux_id)) return 1;
            String ip_str(host);
            return (WiFi.createTCP(mux_id, ip_str, port))? 1 : 0;
        }
        else {
            if (WiFi.isConnected()) return 1;
            String ip_str(host);
            return (WiFi.createTCP(ip_str, port))? 1 : 0;
        }
    };
    
    size_t write(uint8_t data) {
        if (mux) {
            if(WiFi.send(mux_id, &data, 1)){
                return 1;
            }        
            return 0;
        }
        else {
            if(WiFi.send(&data, 1)){
                return 1;
            }        
            return 0;
        }
    };

    size_t write(const uint8_t *buf, size_t size){
        if (mux) {
            if(WiFi.send(mux_id, buf, size)){
                return size;
            }
            return 0;
        }
        else {
            if(WiFi.send(buf, size)){
                return size;
            }
            return 0;            
        }
    };
    
    int available(void){
        if(mux) {
            return WiFi.isDataAvailable(mux_id);
        }
        else {
            return WiFi.isDataAvailable();
        }
    };
    
    int read(){
        uint8_t data;
        if(mux) {
            if(WiFi.recv(mux_id, &data, 1) == 1) {
                return data;
            }
            return (int)-1;            
        }
        else {
            if(WiFi.recv(&data, 1) == 1) {
                return data;
            }
            return (int)-1;
        }
    };
    
    int read(uint8_t *buf, size_t size){
        if(mux) {
            return WiFi.recv(mux_id, buf, size);
        }
        else {
            return WiFi.recv(buf, size);
        }
    };
    
    int peek(void){
        return pbuffer->peek();
    };
    
    void flush(void){};
    void stop(void){
        if(mux) {
            if (WiFi.isConnected(mux_id)) {
                WiFi.releaseTCP(mux_id);
            }            
        }
        else {
            if (WiFi.isConnected()) {
                WiFi.releaseTCP();
            }
        }
    };
    uint8_t connected(void){
        if(mux) {
            return (WiFi.isConnected(mux_id))? 1 : 0;
        }
        else {
            return (WiFi.isConnected())? 1 : 0;
        }
    };
    operator bool(){
        if(mux) {
            return WiFi.isConnected(mux_id);
        }
        else {
            return WiFi.isConnected();
        }
    };

  private:
    ESP8266_RingBuffer *pbuffer;
    uint8_t mux_id;
    bool    mux;
};

#endif /* _CLIENT_ESP8366_H_ */
