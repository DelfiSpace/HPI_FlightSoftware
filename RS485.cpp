/*
 * RS485.cpp
 *
 *  Created on: 28 Dec 2019
 *      Author: stefanosperett
 */

#include <RS485.h>

extern DSerial serial;

volatile unsigned char state = 0;
unsigned short size = 0;
unsigned short index = 0;
PQ9Frame received;

// A pointer list of RS485 objects
RS485 *RS485_instances[4];

void RS485_IRQHandler( unsigned char m )
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus( RS485_instances[m]->module );

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        // new byte received
        unsigned char data = MAP_UART_receiveData( RS485_instances[m]->module );

        switch(state)
        {
            case 0:
                if (data == 0x7E)
                {
                    state = 1;
                    RS485_instances[m]->initCRC();
                }
                break;

            case 1:
                size = (((unsigned short)data) & 0xFF) << 8;
                state = 2;
                break;

            case 2:
                size |= ((unsigned short)data) & 0xFF;
                index = 0;
                if (size > 258)
                {
                    // size is too big, ignore the frame and start over
                    state = 0;
                }
                else
                {
                    // size matched, keep going...
                    state = 3;
                }
                break;

            case 3:
                if (index == 0)
                {
                    received.setDestination(data);
                    RS485_instances[m]->calculateCRC(data);
                }
                if (index == 1)
                {
                    received.setPayloadSize(data);
                    RS485_instances[m]->calculateCRC(data);
                }
                if (index == 2)
                {
                    received.setSource(data);
                    RS485_instances[m]->calculateCRC(data);
                }
                if ((index > 2) && (index <= 258))
                {
                    received.getPayload()[index - 3] = data;
                    RS485_instances[m]->calculateCRC(data);
                }
                if (index > 258)
                {
                    state = 0;
                }
                if (index == size - 1)
                {
                    state = 4;
                }
                index++;
                break;

            case 4:
                // CRC first byte
                if ( data == ((RS485_instances[m]->crc >> 8) & 0xFF) )
                {
                    state = 5;
                }
                else
                {
                    state = 0;
                }
                break;

            case 5:
                // CRC second byte
                if ( data == (RS485_instances[m]->crc & 0xFF) )
                {
                    state = 6;
                }
                else
                {
                    state = 0;
                }
                break;

            case 6:
                // last byte
                if (data == 0x7D)
                {
                    // frame found!
                    RS485_instances[m]->user_onReceive(received);
                }
                state = 0;
                break;

            default:
                state = 0;
        }
    }
}

void RS485_IRQHandler0( void )
{
    RS485_IRQHandler(0);
}

void RS485_IRQHandler1( void )
{
    RS485_IRQHandler(1);
}

void RS485_IRQHandler2( void )
{
    RS485_IRQHandler(2);
}

void RS485_IRQHandler3( void )
{
    RS485_IRQHandler(3);
}

RS485::RS485(uint8_t mod, unsigned long port, unsigned long pin)
{
    switch (mod)
    {
        case 0:
            module = EUSCI_A0_BASE;
            RS485_instances[0] = this;
            break;

        case 1:
            this->module = EUSCI_A1_BASE;
            RS485_instances[1] = this;
            break;

        case 2:
            this->module = EUSCI_A2_BASE;
            RS485_instances[2] = this;
            break;

        case 3:
            this->module = EUSCI_A3_BASE;
            RS485_instances[3] = this;
            break;
    }

    TXEnablePort = port;
    TXEnablePin = pin;
}

void RS485::init( unsigned int baudrate, unsigned char address)
{
    this->baudrate = baudrate;
    this->address = address;

    MAP_UART_disableModule(module);     // disable UART operation for configuration settings

    _initMain();                        // UART pins init

    eUSCI_UART_Config Config;

    //Default Configuration, macro found in uart.h
    Config.selectClockSource    = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    Config.parity               = EUSCI_A_UART_NO_PARITY;
    Config.msborLsbFirst        = EUSCI_A_UART_LSB_FIRST;
    Config.numberofStopBits     = EUSCI_A_UART_ONE_STOP_BIT;
    Config.uartMode             = EUSCI_A_UART_MODE;

    unsigned int n = MAP_CS_getSMCLK() / baudrate;

    if (n > 16)
    {
        Config.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;    // Oversampling
        Config.clockPrescalar = n >> 4;                                         // BRDIV = n / 16
        Config.firstModReg = n - (Config.clockPrescalar << 4);                  // UCxBRF = int((n / 16) - int(n / 16)) * 16
    }
    else
    {
        Config.overSampling = EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;   // Oversampling
        Config.clockPrescalar = n;                                              // BRDIV = n
        Config.firstModReg = 0;                                                 // UCxBRF not used
    }

    Config.secondModReg = 0;                                                    // UCxBRS = 0

    MAP_UART_initModule(module, &Config);
    MAP_UART_enableModule(module);                                              // enable UART operation
}

void RS485::setReceiveHandler( void (*islHandle)(DataFrame &) )
{
    user_onReceive = islHandle; //parse handler function

    if ( islHandle )
    {
        state = 0;

        uint32_t status = MAP_UART_getEnabledInterruptStatus(this->module);

        // clear the receive interrupt to avoid spurious triggers the first time
        MAP_UART_clearInterruptFlag( this->module, status );

        // enable receive interrupt
        MAP_UART_enableInterrupt( this->module, EUSCI_A_UART_RECEIVE_INTERRUPT );
    }
    else
    {
        // disable receive interrupt
        MAP_UART_disableInterrupt( this->module, EUSCI_A_UART_RECEIVE_INTERRUPT );
    }
}

void RS485::transmit( DataFrame &frame )
{
    // calculate the CRC before transmitting the frame
    initCRC();
    calculateCRC(frame.getDestination());
    calculateCRC(frame.getPayloadSize());
    calculateCRC(frame.getSource());
    for (int i = 0; i < frame.getPayloadSize(); i++)
    {
        calculateCRC(frame.getPayload()[i]);
    }

    // Introduce a delay to prevent starting transmission before the write
    // enable of the other side has been turned off (due to the USCI42 errata)
    // introduce a 2 bytes delay to make sure the UART buffer is flushed
    uint32_t d1 = (baudrate == 9600) ? MAP_CS_getMCLK() * 3 / baudrate : MAP_CS_getMCLK() * 4 / baudrate;
    for(uint32_t k = 0; k < d1;  k++)
    {
        __asm("  nop");
    }

    MAP_GPIO_setOutputHighOnPin( TXEnablePort, TXEnablePin );

    MAP_UART_transmitData(module, 0x7E);

    unsigned short size = frame.getPayloadSize() + 3;

    MAP_UART_transmitData(module, (size >> 8) & 0xFF);
    MAP_UART_transmitData(module, size & 0xFF);

    MAP_UART_transmitData(module, frame.getDestination());

    MAP_UART_transmitData(module, frame.getPayloadSize());

    MAP_UART_transmitData(module, frame.getSource());

    for (int i = 0; i < frame.getPayloadSize(); i++)
    {
        MAP_UART_transmitData(module, frame.getPayload()[i]);
    }

    MAP_UART_transmitData(module, (crc >> 8) & 0xFF);
    MAP_UART_transmitData(module, crc & 0xFF);

    MAP_UART_transmitData(module, 0x7D);

    // Workaround for USCI42 errata
    // introduce a 2 bytes delay to make sure the UART buffer is flushed
    uint32_t d = (baudrate == 9600) ? MAP_CS_getMCLK() * 2 / baudrate : MAP_CS_getMCLK() * 3 / baudrate;
    for(uint32_t k = 0; k < d;  k++)
    {
        __asm("  nop");
    }

    MAP_GPIO_setOutputLowOnPin( TXEnablePort, TXEnablePin );
}

void RS485::initCRC()
{
    crc = 0xFFFF;
}

void RS485::calculateCRC(unsigned char data)
{
    for (unsigned int i = 0; i < 8; i++)
    {
        if (((( crc & 0x8000) >> 8) ^ (data & 0x80)) != 0)
        {
            crc <<= 1;
            crc ^= POLY;
        }
        else
        {
            crc <<= 1;
        }
        data <<= 1;
    }
}

void RS485::_initMain( void )
{
    switch (module)
    {
        case EUSCI_A0_BASE:

        modulePort = EUSCI_A0_PORT;
        modulePins = EUSCI_A0_PINS;

        // transmit / receive interrupt request handler
        MAP_UART_registerInterrupt(this->module, RS485_IRQHandler0);

        break;

        case EUSCI_A1_BASE:

        modulePort = EUSCI_A1_PORT;
        modulePins = EUSCI_A1_PINS;

        // transmit / receive interrupt request handler
        MAP_UART_registerInterrupt(this->module, RS485_IRQHandler1);

        break;

        case EUSCI_A2_BASE:

        modulePort = EUSCI_A2_PORT;
        modulePins = EUSCI_A2_PINS;

        // transmit / receive interrupt request handler
        MAP_UART_registerInterrupt(this->module, RS485_IRQHandler2);

        break;

        case EUSCI_A3_BASE:

        modulePort = EUSCI_A3_PORT;
        modulePins = EUSCI_A3_PINS;

        // transmit / receive interrupt request handler
        MAP_UART_registerInterrupt(this->module, RS485_IRQHandler3);

        break;
    }

    // init RXD / TXD pins
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(modulePort, modulePins, GPIO_PRIMARY_MODULE_FUNCTION);

    // init TX enable
    MAP_GPIO_setOutputLowOnPin( TXEnablePort, TXEnablePin );
    MAP_GPIO_setAsOutputPin( TXEnablePort, TXEnablePin );
}

unsigned char RS485::getAddress()
{
    return address;
}
