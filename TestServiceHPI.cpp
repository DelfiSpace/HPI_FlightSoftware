/*
 * TestService.cpp
 *
 *  Created on: 13 Feb 2020
 *      Author: Casper
 */


#include <TestServiceHPI.h>

extern DSerial serial;

void enableLowC(bool status){
    if(status){
        serial.println("Enabling LowC!");
        MAP_GPIO_setOutputHighOnPin( GPIO_PORT_P5, GPIO_PIN0 );
    }else{
        serial.println("Disabling LowC!");
        MAP_GPIO_setOutputLowOnPin( GPIO_PORT_P5, GPIO_PIN0 );
    }
}

void enableHighC(bool status){
    if(status){
        serial.println("Enabling HighC!");
        MAP_GPIO_setOutputHighOnPin( GPIO_PORT_P5, GPIO_PIN6 );
    }else{
        serial.println("Disabling HighC!");
        MAP_GPIO_setOutputLowOnPin( GPIO_PORT_P5, GPIO_PIN6 );
    }
}

void enableLowCShort(bool status){
    if(!status){
        serial.println("Disabling LowCShort!");
        MAP_GPIO_setOutputHighOnPin( GPIO_PORT_P5, GPIO_PIN1 );
    }else{
        serial.println("Enabling LowCShort!");
        MAP_GPIO_setOutputLowOnPin( GPIO_PORT_P5, GPIO_PIN1 );
    }
}

void enableHighCShort(bool status){
    if(!status){
        serial.println("Disabling LowC!");
        MAP_GPIO_setOutputHighOnPin( GPIO_PORT_P5, GPIO_PIN7 );
    }else{
        serial.println("Enabling LowC!");
        MAP_GPIO_setOutputLowOnPin( GPIO_PORT_P5, GPIO_PIN7 );
    }
}

TestServiceHPI::TestServiceHPI(){
    enableLowCShort(false);
    enableHighCShort(false);
    enableLowC(false);
    enableHighC(false);

    MAP_GPIO_setAsOutputPin( GPIO_PORT_P5, GPIO_PIN0 );
    MAP_GPIO_setAsOutputPin( GPIO_PORT_P5, GPIO_PIN1 );
    MAP_GPIO_setAsOutputPin( GPIO_PORT_P5, GPIO_PIN6 );
    MAP_GPIO_setAsOutputPin( GPIO_PORT_P5, GPIO_PIN7 );


    //lowC ADCin:  (P4.4)A9
    //lowC ADCout: (P4.5)A8

    //hiC ADCin: (P5.2)A3
    //hiC ADCin: (P5.3)A2
    /* Initializing ADC (MCLK/1/4) */

}

bool TestServiceHPI::process(DataMessage &command, DataMessage &workingBuffer)
{
    if (command.getPayload()[0] == TESTSERVICEHPI_NR)
    {
        serial.println("TestService - HPI!");
        workingBuffer.setSize(3);
        workingBuffer.getPayload()[0] = TESTSERVICEHPI_NR;
        workingBuffer.getPayload()[1] = SERVICE_RESPONSE_REPLY;
        workingBuffer.getPayload()[2] = 0;

        if (command.getPayload()[1] == SERVICE_RESPONSE_REQUEST)
        {
            serial.print("REQUEST");
            switch(command.getPayload()[2]) {
                case TESTSERVICEHPI_LOWC_ENABLE:
                    enableLowC(command.getPayload()[3] == 1);
                    break;
                case TESTSERVICEHPI_LOWC_TRIGGERSC:
                    enableLowCShort(command.getPayload()[3] == 1);
                    break;
                case TESTSERVICEHPI_HIGHC_ENABLE:
                    enableHighC(command.getPayload()[3] == 1);
                    break;
                case TESTSERVICEHPI_HIGHC_TRIGGERSC:
                    enableHighCShort(command.getPayload()[3] == 1);
                    break;
            }
        }

        return true;
    }
    return false;
}



