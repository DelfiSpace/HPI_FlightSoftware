/*
 * RS485.h
 *
 *  Created on: 28 Dec 2019
 *      Author: stefanosperett
 */

#ifndef RS485_H_
#define RS485_H_

#include <driverlib.h>
#include "PQ9Frame.h"
#include "DSerial.h"

// Device specific includes
#include "inc/msp432p401r.h"

class RS485
{
private:
    uint32_t module;
    unsigned long TXEnablePort;
    unsigned long TXEnablePin;
    uint8_t modulePort;
    uint16_t modulePins;
    unsigned int baudrate;
    void (*user_onReceive)( PQ9Frame & );

    friend void RS485_IRQHandler( unsigned char m );
    friend void RS485_IRQHandler0( void );
    friend void RS485_IRQHandler1( void );
    friend void RS485_IRQHandler2( void );
    friend void RS485_IRQHandler3( void );
    void _initMain( void );

public:
    RS485( uint8_t mod, unsigned long port, unsigned long pin );
    void init( unsigned int baudrate );
    void setReceptionHandler( void (*islHandle)( PQ9Frame & ) );
    void transmit( PQ9Frame &frame );
};

#endif /* RS485_H_ */
