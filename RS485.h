/*
 * RS485.h
 *
 *  Created on: 28 Dec 2019
 *      Author: stefanosperett
 */

#ifndef RS485_H_
#define RS485_H_

#include "DataBus.h"
#include <driverlib.h>
#include "PQ9Frame.h"

// Device specific includes
#include "inc/msp432p401r.h"

#define POLY 0x1021

class RS485 : public DataBus
{
private:
    uint32_t module;
    unsigned long TXEnablePort;
    unsigned long TXEnablePin;
    uint8_t modulePort;
    uint16_t modulePins;
    unsigned int baudrate;
    unsigned char address;
    void (*user_onReceive)( DataFrame & );
    unsigned short crc;

    friend void RS485_IRQHandler( unsigned char m );
    friend void RS485_IRQHandler0( void );
    friend void RS485_IRQHandler1( void );
    friend void RS485_IRQHandler2( void );
    friend void RS485_IRQHandler3( void );
    void initCRC();
    void calculateCRC(unsigned char data);
    void _initMain( void );

public:
    RS485( uint8_t mod, unsigned long port, unsigned long pin );
    void init( unsigned int baudrate, unsigned char address );
    virtual void setReceiveHandler( void (*islHandle)( DataFrame & ) );
    virtual void transmit( DataFrame &frame );
    virtual unsigned char getAddress();
};

#endif /* RS485_H_ */
