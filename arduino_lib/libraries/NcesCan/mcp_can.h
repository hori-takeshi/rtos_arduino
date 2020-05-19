/*
  mcp_can.h
  2012 Copyright (c) Seeed Technology Inc.  All right reserved.

  Author:Loovee
  Contributor: Cory J. Fowler
  2014-1-16
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
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-
  1301  USA
*/
#ifndef _MCP2515_H_
#define _MCP2515_H_

#include "mcp_can_dfs.h"

#define MAX_CHAR_IN_MESSAGE 8

class MCP_CAN
{
    private:
    
    INT8U   SPICS;

/*
*  mcp2515 driver function 
*/
   // private:
private:

    void mcp2515_reset(void);                                           /* reset mcp2515                */

    INT8U mcp2515_readRegister(const INT8U address);                    /* read mcp2515's register      */
    
    void mcp2515_readRegisterS(const INT8U address, 
	                       INT8U values[], 
                               const INT8U n);
    void mcp2515_setRegister(const INT8U address,                       /* set mcp2515's register       */
                             const INT8U value);

    void mcp2515_setRegisterS(const INT8U address,                      /* set mcp2515's registers      */
                              const INT8U values[],
                              const INT8U n);
    
    void mcp2515_initCANBuffers(void);
    
    void mcp2515_modifyRegister(const INT8U address,                    /* set bit of one register      */
                                const INT8U mask,
                                const INT8U data);

    INT8U mcp2515_readStatus(void);                                     /* read mcp2515's Status        */
    INT8U mcp2515_setCANCTRL_Mode(const INT8U newmode);                 /* set mode                     */
    INT8U mcp2515_configRate(const INT8U canSpeed);                     /* set boadrate                 */
    INT8U mcp2515_init(const INT8U canSpeed);                           /* mcp2515init                  */

    void mcp2515_write_id( const INT8U mcp_addr,                        /* write can id                 */
                               const INT8U ext,
                               const INT32U id );

    void mcp2515_read_id( const INT8U mcp_addr,                         /* read can id                  */
                                    INT8U* ext,
                                    INT32U* id );

    void mcp2515_write_canMsg( INT8U txbid, INT32U id, INT8U len, INT8U *pData, INT8U ext, INT8U rtr);          /* write can msg                */
    void mcp2515_read_canMsg( INT8U rxbid, INT32U *p_id, INT8U *p_len, INT8U *p_buf, INT8U *p_ext, INT8U *p_rtr);            /* read can msg                 */
    void mcp2515_start_transmit(INT8U txbid);                  /* start transmit               */
    INT8U mcp2515_getNextFreeTXBuf(INT8U *txbuf_n);                     /* get Next free txbuf          */

public:
    MCP_CAN(INT8U _CS);
    INT8U begin(INT8U speedset);                                    /* init can                     */
    INT8U enableInterrupt(INT8U mask);
    INT8U disableInterrupt(INT8U mask);
    INT8U attachInterrupt(uint32_t ulPin, voidFuncPtr callback);    
    INT8U init_Mask(INT8U num, INT32U ulData, INT8U ext = 0);       /* init Masks                   */
    INT8U init_Filter(INT8U num, INT32U ulData, INT8U ext = 0);       /* init filters                 */    
    INT8U startReceive(INT8U rxbid);                                /* Start Receive                */
    INT8U sendMsg(INT8U txbid, INT32U id, INT8U len, INT8U *p_buf, INT8U ext, INT8U rtr);   /* send msg                     */
    INT8U sendMsg(INT8U txbid, INT32U id, INT8U len, INT8U *p_buf, INT8U ext);   /* send msg                     */
    INT8U sendMsg(INT8U txbid, INT32U id, INT8U len, INT8U *p_buf);   /* send msg                     */    
    INT8U readMsg(INT8U rxbid, INT32U *p_id, INT8U *p_len, INT8U *p_buf, INT8U *p_ext, INT8U *p_rtr);         /* read msg   */
    INT8U readMsg(INT8U rxbid, INT32U *p_id, INT8U *p_len, INT8U *p_buf, INT8U *p_ext);         /* read msg */    
    INT8U readMsg(INT8U rxbid, INT32U *p_id, INT8U *p_len, INT8U *p_buf);         /* read msg */
    INT8U readMsg(INT8U rxbid, INT8U *p_len, INT8U *p_buf);         /* read buf with object ID      */        
    INT8U checkReceive(INT8U rxbid);                                /* if something received        */
    INT8U checkSend(INT8U txbid);                                   /* if something sent            */    
    INT8U checkError(void);                                         /* if something error           */
};

#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
