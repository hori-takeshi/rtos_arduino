/*
  mcp_can.cpp
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
#include "mcp_can.h"

#ifdef TOPPERS_WITH_ARDUINO
#include "r2ca.h"
#define ENTER_CRITICAL wai_sem(SPI_SEM);
#define LEAVE_CRITICAL sig_sem(SPI_SEM);
#else /* !TOPPERS_WITH_ARDUINO */
#define WAIT_TIMEOUT
#define ENTER_CRITICAL
#define LEAVE_CRITICAL
#endif /* TOPPERS_WITH_ARDUINO */

#define spi_readwrite SPI.transfer
#define spi_read() spi_readwrite(0x00)

#define CHECK_RXBID(rxbid) do {				\
	if (!((rxbid == 0) || (rxbid == 1))) {	\
		return CAN_FAIL;					\
	}										\
} while(false)

#define CHECK_LEN(len) do {					\
	if (len > 8) {							\
		return CAN_FAIL;					\
	}										\
} while(false)

#define CHECK_TXBID(txbid) do {								\
	if (!((txbid == 0) || (txbid == 1) || (txbid == 2))) {	\
		return CAN_FAIL;									\
	}														\
} while(false)

/*********************************************************************************************************
** Function name:           mcp2515_reset
** Descriptions:            reset the device
*********************************************************************************************************/
void MCP_CAN::mcp2515_reset(void)
{
    MCP2515_SELECT();
    spi_readwrite(MCP_RESET);
    MCP2515_UNSELECT();
    delay(10);
}

/*********************************************************************************************************
** Function name:           mcp2515_readRegister
** Descriptions:            read register
*********************************************************************************************************/
INT8U MCP_CAN::mcp2515_readRegister(const INT8U address)
{
    INT8U ret;

    MCP2515_SELECT();
    spi_readwrite(MCP_READ);
    spi_readwrite(address);
    ret = spi_read();
    MCP2515_UNSELECT();

    return ret;
}

/*********************************************************************************************************
** Function name:           mcp2515_readRegisterS
** Descriptions:            read registerS
*********************************************************************************************************/
void MCP_CAN::mcp2515_readRegisterS(const INT8U address, INT8U values[], const INT8U n)
{
	INT8U i;
	MCP2515_SELECT();
	spi_readwrite(MCP_READ);
	spi_readwrite(address);
	// mcp2515 has auto-increment of address-pointer
	for (i=0; i<n && i<CAN_MAX_CHAR_IN_MESSAGE; i++) {
		values[i] = spi_read();
	}
	MCP2515_UNSELECT();
}

/*********************************************************************************************************
** Function name:           mcp2515_setRegister
** Descriptions:            set register
*********************************************************************************************************/
void MCP_CAN::mcp2515_setRegister(const INT8U address, const INT8U value)
{
    MCP2515_SELECT();
    spi_readwrite(MCP_WRITE);
    spi_readwrite(address);
    spi_readwrite(value);
    MCP2515_UNSELECT();
}

/*********************************************************************************************************
** Function name:           mcp2515_setRegisterS
** Descriptions:            set registerS
*********************************************************************************************************/
void MCP_CAN::mcp2515_setRegisterS(const INT8U address, const INT8U values[], const INT8U n)
{
    INT8U i;
    MCP2515_SELECT();
    spi_readwrite(MCP_WRITE);
    spi_readwrite(address);
       
    for (i=0; i<n; i++) 
    {
        spi_readwrite(values[i]);
    }
    MCP2515_UNSELECT();
}

/*********************************************************************************************************
** Function name:           mcp2515_modifyRegister
** Descriptions:            set bit of one register
*********************************************************************************************************/
void MCP_CAN::mcp2515_modifyRegister(const INT8U address, const INT8U mask, const INT8U data)
{
    MCP2515_SELECT();
    spi_readwrite(MCP_BITMOD);
    spi_readwrite(address);
    spi_readwrite(mask);
    spi_readwrite(data);
    MCP2515_UNSELECT();
}

/*********************************************************************************************************
** Function name:           mcp2515_readStatus
** Descriptions:            read mcp2515's Status
*********************************************************************************************************/
INT8U MCP_CAN::mcp2515_readStatus(void)                             
{
	INT8U i;
	MCP2515_SELECT();
	spi_readwrite(MCP_READ_STATUS);
	i = spi_read();
	MCP2515_UNSELECT();
	
	return i;
}

/*********************************************************************************************************
** Function name:           mcp2515_setCANCTRL_Mode
** Descriptions:            set control mode
*********************************************************************************************************/
INT8U MCP_CAN::mcp2515_setCANCTRL_Mode(const INT8U newmode)
{
    INT8U i;

    mcp2515_modifyRegister(MCP_CANCTRL, MODE_MASK, newmode);

    i = mcp2515_readRegister(MCP_CANCTRL);
    i &= MODE_MASK;

    if ( i == newmode ) 
    {
        return MCP2515_OK;
    }

    return MCP2515_FAIL;

}

/*********************************************************************************************************
** Function name:           mcp2515_configRate
** Descriptions:            set boadrate
*********************************************************************************************************/
INT8U MCP_CAN::mcp2515_configRate(const INT8U canSpeed)            
{
    INT8U set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canSpeed) 
    {
        case (CAN_5KBPS):
        cfg1 = MCP_16MHz_5kBPS_CFG1;
        cfg2 = MCP_16MHz_5kBPS_CFG2;
        cfg3 = MCP_16MHz_5kBPS_CFG3;
        break;

        case (CAN_10KBPS):
        cfg1 = MCP_16MHz_10kBPS_CFG1;
        cfg2 = MCP_16MHz_10kBPS_CFG2;
        cfg3 = MCP_16MHz_10kBPS_CFG3;
        break;

        case (CAN_20KBPS):
        cfg1 = MCP_16MHz_20kBPS_CFG1;
        cfg2 = MCP_16MHz_20kBPS_CFG2;
        cfg3 = MCP_16MHz_20kBPS_CFG3;
        break;
        
        case (CAN_31K25BPS):
        cfg1 = MCP_16MHz_31k25BPS_CFG1;
        cfg2 = MCP_16MHz_31k25BPS_CFG2;
        cfg3 = MCP_16MHz_31k25BPS_CFG3;
        break;

        case (CAN_33KBPS):
        cfg1 = MCP_16MHz_33kBPS_CFG1;
        cfg2 = MCP_16MHz_33kBPS_CFG2;
        cfg3 = MCP_16MHz_33kBPS_CFG3;
        break;

        case (CAN_40KBPS):
        cfg1 = MCP_16MHz_40kBPS_CFG1;
        cfg2 = MCP_16MHz_40kBPS_CFG2;
        cfg3 = MCP_16MHz_40kBPS_CFG3;
        break;

        case (CAN_50KBPS):
        cfg1 = MCP_16MHz_50kBPS_CFG1;
        cfg2 = MCP_16MHz_50kBPS_CFG2;
        cfg3 = MCP_16MHz_50kBPS_CFG3;
        break;

        case (CAN_80KBPS):
        cfg1 = MCP_16MHz_80kBPS_CFG1;
        cfg2 = MCP_16MHz_80kBPS_CFG2;
        cfg3 = MCP_16MHz_80kBPS_CFG3;
        break;

        case (CAN_83K3BPS):
        cfg1 = MCP_16MHz_83k3BPS_CFG1;
        cfg2 = MCP_16MHz_83k3BPS_CFG2;
        cfg3 = MCP_16MHz_83k3BPS_CFG3;
        break;  

        case (CAN_95KBPS):
        cfg1 = MCP_16MHz_95kBPS_CFG1;
        cfg2 = MCP_16MHz_95kBPS_CFG2;
        cfg3 = MCP_16MHz_95kBPS_CFG3;
        break;

        case (CAN_100KBPS):                                             /* 100KBPS                  */
        cfg1 = MCP_16MHz_100kBPS_CFG1;
        cfg2 = MCP_16MHz_100kBPS_CFG2;
        cfg3 = MCP_16MHz_100kBPS_CFG3;
        break;

        case (CAN_125KBPS):
        cfg1 = MCP_16MHz_125kBPS_CFG1;
        cfg2 = MCP_16MHz_125kBPS_CFG2;
        cfg3 = MCP_16MHz_125kBPS_CFG3;
        break;

        case (CAN_200KBPS):
        cfg1 = MCP_16MHz_200kBPS_CFG1;
        cfg2 = MCP_16MHz_200kBPS_CFG2;
        cfg3 = MCP_16MHz_200kBPS_CFG3;
        break;

        case (CAN_250KBPS):
        cfg1 = MCP_16MHz_250kBPS_CFG1;
        cfg2 = MCP_16MHz_250kBPS_CFG2;
        cfg3 = MCP_16MHz_250kBPS_CFG3;
        break;

        case (CAN_500KBPS):
        cfg1 = MCP_16MHz_500kBPS_CFG1;
        cfg2 = MCP_16MHz_500kBPS_CFG2;
        cfg3 = MCP_16MHz_500kBPS_CFG3;
        break;
        
        case (CAN_1000KBPS):
        cfg1 = MCP_16MHz_1000kBPS_CFG1;
        cfg2 = MCP_16MHz_1000kBPS_CFG2;
        cfg3 = MCP_16MHz_1000kBPS_CFG3;
        break;  

        default:
        set = 0;
        break;
    }

    if (set) {
        mcp2515_setRegister(MCP_CNF1, cfg1);
        mcp2515_setRegister(MCP_CNF2, cfg2);
        mcp2515_setRegister(MCP_CNF3, cfg3);
        return MCP2515_OK;
    }
    else {
        return MCP2515_FAIL;
    }
}

/*********************************************************************************************************
** Function name:           mcp2515_initCANBuffers
** Descriptions:            init canbuffers
*********************************************************************************************************/
void MCP_CAN::mcp2515_initCANBuffers(void)
{
    INT8U i, a1, a2, a3;
    
    INT8U std = 0;               
    INT8U ext = 1;
    INT32U ulMask = 0xffffffff, ulFilt = 0x00;


    mcp2515_write_id(MCP_RXM0SIDH, ext, ulMask);			/*Set both masks to 0xff       */
    mcp2515_write_id(MCP_RXM1SIDH, ext, ulMask);			/*Mask register ignores ext bit */
    
                                                            /* Set all filters to 0         */
    mcp2515_write_id(MCP_RXF0SIDH, std, ulFilt);			/* RXB0: standard               */
    mcp2515_write_id(MCP_RXF1SIDH, std, ulFilt);			/* RXB1: standard               */
    mcp2515_write_id(MCP_RXF2SIDH, std, ulFilt);			/* RXB2: standard               */
    mcp2515_write_id(MCP_RXF3SIDH, std, ulFilt);			/* RXB3: standard               */
    mcp2515_write_id(MCP_RXF4SIDH, std, ulFilt);
    mcp2515_write_id(MCP_RXF5SIDH, std, ulFilt);
    
                                                                        /* Clear, deactivate the three  */
                                                                        /* transmit buffers             */
                                                                        /* TXBnCTRL -> TXBnD7           */
    a1 = MCP_TXB0CTRL;
    a2 = MCP_TXB1CTRL;
    a3 = MCP_TXB2CTRL;
    for (i = 0; i < 14; i++) {                                          /* in-buffer loop               */
        mcp2515_setRegister(a1, 0);
        mcp2515_setRegister(a2, 0);
        mcp2515_setRegister(a3, 0);
        a1++;
        a2++;
        a3++;
    }
    mcp2515_setRegister(MCP_RXB0CTRL, 0x00);
    mcp2515_setRegister(MCP_RXB1CTRL, 0xff);
}

/*********************************************************************************************************
** Function name:           mcp2515_init
** Descriptions:            init the device
*********************************************************************************************************/
INT8U MCP_CAN::mcp2515_init(const INT8U canSpeed)                       /* mcp2515init                  */
{

  INT8U res;

    mcp2515_reset();

    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if(res > 0)
    {
#if DEBUG_MODE
      Serial.print("Enter setting mode fall\r\n"); 
#endif
      return res;
    }
#if DEBUG_MODE
    Serial.print("Enter setting mode success \r\n");
#endif

    if(mcp2515_configRate(canSpeed))
    {
#if DEBUG_MODE
      Serial.print("set rate fall!!\r\n");
#endif
      return res;
    }
#if DEBUG_MODE
    Serial.print("set rate success!!\r\n");
#endif

    if ( res == MCP2515_OK ) {

                                                                        /* init canbuffers              */
        mcp2515_initCANBuffers();

                                                                        /* interrupt mode               */
        mcp2515_setRegister(MCP_CANINTE, 0x00);
        
                                                                        /* enter normal mode            */
        res = mcp2515_setCANCTRL_Mode(MODE_NORMAL);                                                                
        if(res)
        {
#if DEBUG_MODE        
          Serial.print("Enter Normal Mode Fall!!\r\n");
#endif           
          return res;
        }


#if DEBUG_MODE
          Serial.print("Enter Normal Mode Success!!\r\n");
#endif

    }
    return res;

}

/*********************************************************************************************************
** Function name:           mcp2515_write_id
** Descriptions:            write can id
*********************************************************************************************************/
void MCP_CAN::mcp2515_write_id( const INT8U mcp_addr, const INT8U ext, const INT32U id )
{
    uint16_t canid;
    INT8U tbufdata[4];

    canid = (uint16_t)(id & 0x0FFFF);

    if ( ext == 1) 
    {
        tbufdata[MCP_EID0] = (INT8U) (canid & 0xFF);
        tbufdata[MCP_EID8] = (INT8U) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (INT8U) (canid & 0x03);
        tbufdata[MCP_SIDL] += (INT8U) ((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (INT8U) (canid >> 5 );
    }
    else 
    {
        tbufdata[MCP_SIDH] = (INT8U) (canid >> 3 );
        tbufdata[MCP_SIDL] = (INT8U) ((canid & 0x07 ) << 5);
        tbufdata[MCP_EID0] = 0;
        tbufdata[MCP_EID8] = 0;
    }
    mcp2515_setRegisterS( mcp_addr, tbufdata, 4 );
}

/*********************************************************************************************************
** Function name:           mcp2515_read_id
** Descriptions:            read can id
*********************************************************************************************************/
void MCP_CAN::mcp2515_read_id( const INT8U mcp_addr, INT8U* ext, INT32U* id )
{
    INT8U tbufdata[4];

    *ext = 0;
    *id = 0;

    mcp2515_readRegisterS( mcp_addr, tbufdata, 4 );

    *id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

    if ( (tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) ==  MCP_TXB_EXIDE_M ) 
    {
                                                                        /* extended id                  */
        *id = (*id<<2) + (tbufdata[MCP_SIDL] & 0x03);
        *id = (*id<<8) + tbufdata[MCP_EID8];
        *id = (*id<<8) + tbufdata[MCP_EID0];
        *ext = 1;
    }
}

/*********************************************************************************************************
** Function name:           mcp2515_write_canMsg
** Descriptions:            write msg
*********************************************************************************************************/
void MCP_CAN::mcp2515_write_canMsg( INT8U txbid, INT32U id, INT8U len, INT8U *pData, INT8U ext, INT8U rtr )
{
    INT8U ctrlregs[MCP_N_TXBUFFERS] = { MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL };    
    INT8U mcp_addr = ctrlregs[txbid] + 1;

    mcp2515_setRegisterS(mcp_addr+5, pData, len );                  /* write data bytes             */
    if ( rtr == 1)                                               /* if RTR set bit in byte       */
    {
        len |= MCP_RTR_MASK;  
    }
    mcp2515_setRegister((mcp_addr+4), len );                        /* write the RTR and DLC        */
    mcp2515_write_id(mcp_addr, ext, id );                           /* write CAN id                 */

}

/*********************************************************************************************************
** Function name:           mcp2515_read_canMsg
** Descriptions:            read message
*********************************************************************************************************/
void MCP_CAN::mcp2515_read_canMsg( INT8U rxbid, INT32U *p_id, INT8U *p_len, INT8U *p_buf, INT8U *p_ext, INT8U *p_rtr)        /* read can msg                 */
{
    INT8U mcp_addr = (rxbid == 0)? MCP_RXBUF_0 : MCP_RXBUF_1;
    INT8U ctrl;

    mcp2515_read_id( mcp_addr, p_ext, p_id );

    ctrl = mcp2515_readRegister( mcp_addr-1 );
    *p_len = mcp2515_readRegister( mcp_addr+4 );

    if ((ctrl & 0x08)) {
        *p_rtr = 1;
    }
    else {
        *p_rtr = 0;
    }

    *p_len &= MCP_DLC_MASK;
    mcp2515_readRegisterS( mcp_addr+5, p_buf, *p_len );
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            send message
*********************************************************************************************************/
void MCP_CAN::mcp2515_start_transmit(INT8U txbid)              /* start transmit               */
{
    INT8U ctrlregs[MCP_N_TXBUFFERS] = { MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL };
    INT8U mcp_addr = ctrlregs[txbid];
    
    mcp2515_modifyRegister( mcp_addr, MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M );
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            send message
*********************************************************************************************************/
INT8U MCP_CAN::mcp2515_getNextFreeTXBuf(INT8U *txbuf_n)                 /* get Next free txbuf          */
{
    INT8U res, i, ctrlval;
    INT8U ctrlregs[MCP_N_TXBUFFERS] = { MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL };

    res = MCP_ALLTXBUSY;
    *txbuf_n = 0x00;

                                                                        /* check all 3 TX-Buffers       */
    for (i=0; i<MCP_N_TXBUFFERS; i++) {
        ctrlval = mcp2515_readRegister( ctrlregs[i] );
        if ( (ctrlval & MCP_TXB_TXREQ_M) == 0 ) {
            *txbuf_n = ctrlregs[i]+1;                                   /* return SIDH-address of Buffe */
                                                                        /* r                            */
            res = MCP2515_OK;
            return res;                                                 /* ! function exit              */
        }
    }
    return res;
}

/*********************************************************************************************************
** Function name:           set CS
** Descriptions:            init CS pin and set UNSELECTED
*********************************************************************************************************/
MCP_CAN::MCP_CAN(INT8U _CS)
{
    SPICS = _CS;
    pinMode(SPICS, OUTPUT);
    MCP2515_UNSELECT();
}

/*********************************************************************************************************
** Function name:           init
** Descriptions:            init can and set speed
*********************************************************************************************************/
INT8U MCP_CAN::begin(INT8U speedset)
{
    INT8U res;
    
    ENTER_CRITICAL
    SPI.begin();
    res = mcp2515_init(speedset);
    LEAVE_CRITICAL
      
    if (res == MCP2515_OK) return CAN_OK;
    else return CAN_FAILINIT;
}

/*********************************************************************************************************
** Function name:           enableInterrupt
** Descriptions:            Enable Interrupt
*********************************************************************************************************/
INT8U MCP_CAN::enableInterrupt(INT8U mask)
{
    ENTER_CRITICAL
    mcp2515_modifyRegister(MCP_CANINTE, mask, mask);
    LEAVE_CRITICAL
      
    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           disableInterrupt
** Descriptions:            Disable Interrupt
*********************************************************************************************************/
INT8U MCP_CAN::disableInterrupt(INT8U mask)
{
    ENTER_CRITICAL
    mcp2515_modifyRegister(MCP_CANINTE, mask, !mask);
    LEAVE_CRITICAL
      
    return CAN_OK;    
}

/*********************************************************************************************************
** Function name:           attachInterrupt
** Descriptions:            Attach Interrupt
*********************************************************************************************************/
INT8U MCP_CAN::attachInterrupt(uint32_t ulPin, voidFuncPtr callback)
{
    ::attachInterrupt( ulPin, callback, FALLING ) ;
    return CAN_OK;    
}

/*********************************************************************************************************
** Function name:           init_Mask
** Descriptions:            init canid Masks
*********************************************************************************************************/
INT8U MCP_CAN::init_Mask(INT8U num, INT32U ulData, INT8U ext)
{
    INT8U res = MCP2515_OK;
    
    ENTER_CRITICAL;
#if DEBUG_MODE
    Serial.print("Begin to set Mask!!\r\n");
#endif
    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if(res > 0){
#if DEBUG_MODE
    Serial.print("Enter setting mode fall\r\n"); 
#endif
    LEAVE_CRITICAL;
    return res;
    }
    
    if (num == 0){
        mcp2515_write_id(MCP_RXM0SIDH, ext, ulData);
    }
    else if(num == 1){
        mcp2515_write_id(MCP_RXM1SIDH, ext, ulData);
    }
    else res =  MCP2515_FAIL;
    
    res = mcp2515_setCANCTRL_Mode(MODE_NORMAL);
    if(res > 0){
#if DEBUG_MODE
    Serial.print("Enter normal mode fall\r\n"); 
#endif
    LEAVE_CRITICAL;        
    return res;
  }
#if DEBUG_MODE
    Serial.print("set Mask success!!\r\n");
#endif
    LEAVE_CRITICAL;
    return res;
}

/*********************************************************************************************************
** Function name:           init_Filter
** Descriptions:            init canid filters
*********************************************************************************************************/
INT8U MCP_CAN::init_Filter(INT8U num, INT32U ulData, INT8U ext)
{
    INT8U res = MCP2515_OK;

    ENTER_CRITICAL;    
#if DEBUG_MODE
    Serial.print("Begin to set Filter!!\r\n");
#endif
    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if(res > 0)
    {
#if DEBUG_MODE
      Serial.print("Enter setting mode fall\r\n"); 
#endif
      LEAVE_CRITICAL;
      return res;
    }
    
    switch( num )
    {
        case 0:
        mcp2515_write_id(MCP_RXF0SIDH, ext, ulData);
        break;

        case 1:
        mcp2515_write_id(MCP_RXF1SIDH, ext, ulData);
        break;

        case 2:
        mcp2515_write_id(MCP_RXF2SIDH, ext, ulData);
        break;

        case 3:
        mcp2515_write_id(MCP_RXF3SIDH, ext, ulData);
        break;

        case 4:
        mcp2515_write_id(MCP_RXF4SIDH, ext, ulData);
        break;

        case 5:
        mcp2515_write_id(MCP_RXF5SIDH, ext, ulData);
        break;

        default:
        res = MCP2515_FAIL;
    }
    
    res = mcp2515_setCANCTRL_Mode(MODE_NORMAL);
    if(res > 0)
    {
#if DEBUG_MODE
      Serial.print("Enter normal mode fall\r\nSet filter fail!!\r\n"); 
#else
      delay(10);
#endif
      LEAVE_CRITICAL;        
      return res;
    }
#if DEBUG_MODE
    Serial.print("set Filter success!!\r\n");
#else
    delay(10);
#endif
    LEAVE_CRITICAL;
    return res;
}

/*********************************************************************************************************
** Function name:           startReceive
** Descriptions:            start Receive 
*********************************************************************************************************/
INT8U MCP_CAN::startReceive(INT8U rxbid)
{
    CHECK_RXBID(rxbid);

    ENTER_CRITICAL;
#if (DEBUG_RXANY==1)
                                                                        /* enable both receive-buffers  */
                                                                        /* to receive any message       */
                                                                        /* and enable rollover          */
    if (rxbid == 0) {
        mcp2515_modifyRegister(MCP_RXB0CTRL,
        MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
        MCP_RXB_RX_ANY);
    }else{
        mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
        MCP_RXB_RX_ANY);
    }
#else
                                                                        /* enable both receive-buffers  */
                                                                        /* to receive messages          */
                                                                        /* with std. and ext. identifie */
                                                                        /* rs                           */
                                                                        /* and enable rollover          */
    if (rxbid == 0) {        
        mcp2515_modifyRegister(MCP_RXB0CTRL,
        MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
        MCP_RXB_RX_STDEXT);
    }else{
        mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
        MCP_RXB_RX_STDEXT);
    }
#endif
    LEAVE_CRITICAL;
    
    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            send message
*********************************************************************************************************/
INT8U MCP_CAN::sendMsg(INT8U txbid, INT32U id, INT8U len, INT8U *p_buf, INT8U ext, INT8U rtr)
{    
    INT8U intf_tbl[MCP_N_TXBUFFERS] = { MCP_TX0IF, MCP_TX1IF, MCP_TX2IF };
    INT8U intf_bit = intf_tbl[txbid];

    CHECK_LEN(len);
    CHECK_TXBID(txbid);
    
    ENTER_CRITICAL;
    mcp2515_write_canMsg(txbid, id, len, p_buf, ext, rtr);
    mcp2515_modifyRegister(MCP_CANINTF, intf_bit, 0);
    mcp2515_start_transmit( txbid );
    LEAVE_CRITICAL;
    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            send buf
*********************************************************************************************************/
INT8U MCP_CAN::sendMsg(INT8U txbid, INT32U id, INT8U len, INT8U *p_buf, INT8U ext)
{
    return sendMsg(txbid, id, len, p_buf, ext, 0);
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            send buf
*********************************************************************************************************/
INT8U MCP_CAN::sendMsg(INT8U txbid, INT32U id, INT8U len, INT8U *p_buf)
{
    return sendMsg(txbid, id, len, p_buf, 0, 0);
}


/*********************************************************************************************************
** Function name:           readMsg
** Descriptions:            read message
*********************************************************************************************************/
INT8U MCP_CAN::readMsg(INT8U rxbid, INT32U *p_id, INT8U *p_len, INT8U *p_buf, INT8U *p_ext, INT8U *p_rtr)
{
    INT8U stat, res;
    CHECK_RXBID(rxbid);
    INT8U stat_bit = (rxbid == 0)? MCP_STAT_RX0IF : MCP_STAT_RX1IF;
    INT8U intf_bit = (rxbid == 0)? MCP_RX0IF : MCP_RX1IF;
    
    ENTER_CRITICAL;      
    stat = mcp2515_readStatus();

    if ( stat & stat_bit )
    {
        mcp2515_read_canMsg(rxbid, p_id, p_len, p_buf, p_ext, p_rtr);
        mcp2515_modifyRegister(MCP_CANINTF, intf_bit, 0);
        res = CAN_OK;
    }
    else {
        res = CAN_NOMSG;
    }
    LEAVE_CRITICAL;    
    return res;
}

/*********************************************************************************************************
** Function name:           readMsgBuf
** Descriptions:            read message
*********************************************************************************************************/
INT8U MCP_CAN::readMsg(INT8U rxbid, INT32U *p_id, INT8U *p_len, INT8U *p_buf, INT8U *p_ext)
{
    INT8U rtr;
    
    return readMsg(rxbid, p_id, p_len, p_buf, p_ext, &rtr);
}

/*********************************************************************************************************
** Function name:           readMsgBuf
** Descriptions:            read message
*********************************************************************************************************/
INT8U MCP_CAN::readMsg(INT8U rxbid, INT32U *p_id, INT8U *p_len, INT8U *p_buf)
{
    INT8U ext;
    INT8U rtr;
    
    return readMsg(rxbid, p_id, p_len, p_buf, &ext, &rtr);
}

/*********************************************************************************************************
** Function name:           readMsgBuf
** Descriptions:            read message
*********************************************************************************************************/
INT8U MCP_CAN::readMsg(INT8U rxbid, INT8U *p_len, INT8U *p_buf)
{
    INT32U id;
    INT8U ext;
    INT8U rtr;
    
    return readMsg(rxbid, &id, p_len, p_buf, &ext, &rtr);
}

/*********************************************************************************************************
** Function name:           checkReceive
** Descriptions:            check if got something
*********************************************************************************************************/
INT8U MCP_CAN::checkReceive(INT8U rxbid)
{
    INT8U stat, res = CAN_NOMSG;
    CHECK_RXBID(rxbid);
    
    ENTER_CRITICAL;
    stat = mcp2515_readStatus();                                         /* RXnIF in Bit 1 and 0         */
    if (rxbid == 0) {
        if ( (stat & MCP_STAT_RX0IF) == MCP_STAT_RX0IF) {
            res = CAN_MSGAVAIL;
        }
    }
    else if (rxbid == 1) {
        if ( (stat & MCP_STAT_RX1IF) == MCP_STAT_RX1IF ) {
            res = CAN_MSGAVAIL;
        }
    }
    LEAVE_CRITICAL;
    
    return res;
}

/*********************************************************************************************************
** Function name:           checkSend
** Descriptions:            check if snt msg
*********************************************************************************************************/
INT8U MCP_CAN::checkSend(INT8U txbid)
{
    INT8U stat, res = CAN_NOSENDWAIT;
    INT8U txbx_ctrl[MCP_N_TXBUFFERS] = { MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL };
    
    CHECK_TXBID(txbid);
    
    ENTER_CRITICAL;    
    stat = mcp2515_readRegister(txbx_ctrl[txbid]);

    if ( (stat & MCP_TXB_TXREQ_M) == MCP_TXB_TXREQ_M) {
        res = CAN_SENDWAIT;
    }
    LEAVE_CRITICAL;
    
    return res;
}

/*********************************************************************************************************
** Function name:           checkError
** Descriptions:            if something error
*********************************************************************************************************/
INT8U MCP_CAN::checkError(void)
{
    INT8U res = CAN_OK;
    
    ENTER_CRITICAL;        
    INT8U eflg = mcp2515_readRegister(MCP_EFLG);

    if ( eflg & MCP_EFLG_ERRORMASK ) 
    {
        res = CAN_CTRLERROR;
    }
    LEAVE_CRITICAL;

    return res;
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
