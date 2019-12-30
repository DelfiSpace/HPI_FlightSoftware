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
                }
                break;

            case 1:
                size = (((unsigned short)data) & 0xFF) << 8;
                state = 2;
                break;

            case 2:
                size |= ((unsigned short)data) & 0xFF;
                index = 0;
                state = 3;
                break;

            case 3:
                if (index == 0)
                {
                    received.setDestination(data);
                }
                if (index == 1)
                {
                    received.setPayloadSize(data);
                }
                if (index == 2)
                {
                    received.setSource(data);
                }
                if ((index > 2) && (index < 258))
                {
                    received.getPayload()[index - 3] = data;
                }
                if (index >= 258)
                {
                    state = 0;
                }
                if (index == size - 2)
                {
                    state = 4;
                }
                index++;
                break;

            case 4:
                // first byte CRC
                status = 5;
                break;

            case 5:
                // second byte CRC
                status = 6;
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
    //RS485_IRQHandler(3);
    uint32_t status = MAP_UART_getEnabledInterruptStatus( RS485_instances[3]->module );

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        // new byte received
        unsigned char data = MAP_UART_receiveData( RS485_instances[3]->module );

        serial.print(data, HEX);
        serial.print(" ");
        serial.print(index, DEC);
        serial.println();

        switch(state)
        {
            case 0:
                if (data == 0x7E)
                {
                    state = 1;
                }
                break;

            case 1:
                size = (((unsigned short)data) & 0xFF) << 8;
                state = 2;
                break;

            case 2:
                size |= ((unsigned short)data) & 0xFF;
                index = 0;
                state = 3;
                break;

            case 3:
                if (index == 0)
                {
                    received.setDestination(data);
                    serial.print("D");
                    serial.print(data, DEC);
                    serial.println();
                }
                if (index == 1)
                {
                    received.setPayloadSize(data);
                    serial.print("S");
                    serial.print(data, DEC);
                    serial.println();
                }
                if (index == 2)
                {
                    received.setSource(data);
                    serial.print("s");
                    serial.print(data, DEC);
                    serial.println();
                }
                if ((index > 2) && (index < 258))
                {
                    received.getPayload()[index - 3] = data;
                }
                if (index >= 258)
                {
                    state = 0;
                }
                if (index == size - 3)
                {
                    state = 4;
                }
                index++;
                break;

            case 4:
                // first byte CRC
                state = 5;
                break;

            case 5:
                // second byte CRC
                state = 6;
                break;

            case 6:
                // last byte
                if (data == 0x7D)
                {
                    serial.println("F");

                    // frame found!
                    RS485_instances[3]->user_onReceive(received);
                }
                state = 0;
                break;

            default:
                state = 0;
        }
    }
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

void RS485::init(unsigned int baudrate)
{
    this->baudrate = baudrate;

    MAP_UART_disableModule(module);   //disable UART operation for configuration settings

    _initMain();    //UART pins init

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

void RS485::setReceptionHandler( void (*islHandle)(PQ9Frame &) )
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
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(modulePort, modulePins, GPIO_PRIMARY_MODULE_FUNCTION);

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin( TXEnablePort,
                                        TXEnablePin, GPIO_PRIMARY_MODULE_FUNCTION );

    MAP_GPIO_setOutputLowOnPin( TXEnablePort, TXEnablePin );

    MAP_GPIO_setAsOutputPin( TXEnablePort, TXEnablePin );
}
