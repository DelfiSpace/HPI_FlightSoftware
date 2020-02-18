/*
 * TestService.cpp
 *
 *  Created on: 13 Feb 2020
 *      Author: Casper
 */


#include <TestServiceHPI.h>
#define nrOfMeas 50

uint16_t outputVoltages[nrOfMeas];

volatile bool DMADone = false;

extern DSerial serial;

void DMA_INT1_IRQHandler(void)
{
    //serial.println("DMA IRQ!");
    MAP_DMA_disableChannel(7);
    MAP_DMA_clearInterruptFlag(7);

    MAP_Timer_A_stopTimer(TIMER_A0_BASE);


    DMADone = true;
}


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
        //serial.println("Enabling LowCShort!");
        MAP_GPIO_setOutputLowOnPin( GPIO_PORT_P5, GPIO_PIN1 );
    }
}

void enableHighCShort(bool status){
    if(!status){
        serial.println("Disabling HighCShort!");
        MAP_GPIO_setOutputHighOnPin( GPIO_PORT_P5, GPIO_PIN7 );
    }else{
        //serial.println("Enabling HighC Short!");
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
    /* Setting reference voltage to 2.5  and enabling reference */
    MAP_REF_A_setReferenceVoltage(REF_A_VREF2_5V);
    MAP_REF_A_enableReferenceVoltage();

    /* Initializing ADC (MCLK/1/4) */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1,
                0);

    /* Configuring GPIOs (4.4 A9) */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4 | GPIO_PIN5,
    GPIO_TERTIARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN2 | GPIO_PIN3,
        GPIO_TERTIARY_MODULE_FUNCTION);

    /* Configuring ADC Memory */
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM3, true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_INTBUF_VREFNEG_VSS,
    ADC_INPUT_A9, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_INTBUF_VREFNEG_VSS,
    ADC_INPUT_A8, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM2, ADC_VREFPOS_INTBUF_VREFNEG_VSS,
    ADC_INPUT_A3, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM3, ADC_VREFPOS_INTBUF_VREFNEG_VSS,
    ADC_INPUT_A2, false);

    /* Enabling interrupts */
    MAP_ADC14_enableInterrupt(ADC_INT3);

    /* Configuring Sample Timer */
    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);
    MAP_ADC14_setSampleHoldTime(ADC_PULSE_WIDTH_96,ADC_PULSE_WIDTH_96);
    MAP_ADC14_setSampleHoldTrigger(ADC_TRIGGER_ADCSC, false); // ADC_TRIGGER_SOURCE1

    /* Enabling/Toggling Conversion */
    MAP_ADC14_enableConversion();
    //MAP_ADC14_toggleConversionTrigger();
    //![Single Sample Mode Configure]
    MAP_CS_initClockSignal(CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_4);


}

bool TestServiceHPI::process(DataMessage &command, DataMessage &workingBuffer)
{
    if (command.getPayload()[0] == TESTSERVICEHPI_NR)
    {
        //serial.println("TestService - HPI!");
        workingBuffer.setSize(3);
        workingBuffer.getPayload()[0] = TESTSERVICEHPI_NR;
        workingBuffer.getPayload()[1] = SERVICE_RESPONSE_REPLY;
        workingBuffer.getPayload()[2] = 0;

        if (command.getPayload()[1] == SERVICE_RESPONSE_REQUEST)
        {
            //serial.print("REQUEST");
            uint32_t d = 0;
            uint64_t status;
            uint16_t inputVoltage;
            uint16_t outputVoltage;
            Timer_A_ContinuousModeConfig  timerConfig;
            Timer_A_ContinuousModeConfig  slowtimerConfig;
            //uint16_t outputVoltages[nrOfMeas+10];

            switch(command.getPayload()[2]) {
                case TESTSERVICEHPI_LOWC_ENABLE:
                    MAP_ADC14_disableConversion();
                    MAP_ADC14_clearInterruptFlag( MAP_ADC14_getEnabledInterruptStatus() );
                    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, true);
                    MAP_ADC14_enableInterrupt(ADC_INT1);
                    MAP_ADC14_enableConversion();


                    enableLowC(command.getPayload()[3] == 1);
                    this->lowCStatus = (command.getPayload()[3] == 1);
                    //wait for voltage Settle
                    d = 0.05 * MAP_CS_getMCLK();
                    for(uint32_t k = 0; k < d;  k++)
                    {
                        __asm("  nop");
                    }
                    //Trigger ADC Conversion!
                    MAP_ADC14_toggleConversionTrigger();

                    //wait for ADC Result
                    status = MAP_ADC14_getEnabledInterruptStatus();
                    while(!(ADC_INT1 & status)){
                        serial.print("waiting!");
                        status = MAP_ADC14_getEnabledInterruptStatus();;
                    }
                    MAP_ADC14_clearInterruptFlag(status);

                    inputVoltage = MAP_ADC14_getResult(ADC_MEM0)* 5000 / 16384;
                    outputVoltage = MAP_ADC14_getResult(ADC_MEM1)* 5000 / 16384;

                    serial.print("INTPUT: ");
                    serial.print(inputVoltage, DEC);
                    serial.print("  |  OUTPUT: ");
                    serial.print(outputVoltage, DEC);
                    serial.println();

                    workingBuffer.getPayload()[2] = (uint8_t) ((inputVoltage & 0xFF00) >> 8);
                    workingBuffer.getPayload()[3] = (uint8_t) (inputVoltage & 0x00FF);
                    workingBuffer.getPayload()[4] = (uint8_t) ((outputVoltage & 0xFF00) >> 8);
                    workingBuffer.getPayload()[5] = (uint8_t) (outputVoltage & 0x00FF);
                    workingBuffer.setSize(6);
                    break;

                case TESTSERVICEHPI_LOWC_TRIGGERSC:
                    if(command.getPayload()[3] == 1 && lowCStatus == true){
                        uint8_t controlTable[16];
                        for(int j = 0; j < 16; j++){
                            controlTable[j] = 0;
                        }
                        //configure Timer
                        timerConfig =
                        {
                                TIMER_A_CLOCKSOURCE_SMCLK,           // SMCLK Clock Source (48Mhz/4)
                                TIMER_A_CLOCKSOURCE_DIVIDER_1,       // SMCLK/1 = 12MHz
                                TIMER_A_TAIE_INTERRUPT_DISABLE,      // Disable Timer ISR
                                TIMER_A_DO_CLEAR                     // Do Clear Counter
                        };
                        slowtimerConfig =
                        {
                                TIMER_A_CLOCKSOURCE_ACLK,           //  ACLK Clock Source (32768 hz)
                                TIMER_A_CLOCKSOURCE_DIVIDER_1,       // ACLK/2 = 16384
                                TIMER_A_TAIE_INTERRUPT_DISABLE,      // Disable Timer ISR
                                TIMER_A_DO_CLEAR                     // Do Clear Counter
                        };
                        MAP_Timer_A_configureContinuousMode(TIMER_A0_BASE, &timerConfig);
                        MAP_Timer_A_configureContinuousMode(TIMER_A1_BASE, &slowtimerConfig);

                        //reconfigure ADC
                        MAP_ADC14_disableConversion();
                        MAP_ADC14_clearInterruptFlag( MAP_ADC14_getEnabledInterruptStatus() );
                        MAP_ADC14_configureSingleSampleMode(ADC_MEM1, true);
                        MAP_ADC14_enableInterrupt(ADC_INT1);
                        MAP_DMA_enableModule();
                        MAP_DMA_setControlBase(controlTable);
                        MAP_DMA_assignChannel(DMA_CH7_ADC14);
                        MAP_DMA_setChannelControl(UDMA_PRI_SELECT | DMA_CH7_ADC14,
                            UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
                        MAP_DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CH7_ADC14,
                            UDMA_MODE_BASIC, (void*) &ADC14->MEM[1],
                            outputVoltages, nrOfMeas);
                        MAP_DMA_assignInterrupt(DMA_INT1, 7);
                        MAP_DMA_registerInterrupt(DMA_INT1, DMA_INT1_IRQHandler);
                        MAP_Interrupt_enableInterrupt(INT_DMA_INT1);

                        DMADone = false;

                        MAP_DMA_enableChannel(7);
                        MAP_ADC14_enableConversion();
                        enableLowCShort(command.getPayload()[3] == 1);
                        MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);
                        MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);
                        MAP_ADC14_toggleConversionTrigger();

                        while(!DMADone);
                        //serial.println("DMA DONE!");

                        uint16_t measTime = MAP_Timer_A_getCounterValue(TIMER_A0_BASE);
                        measTime = measTime/(MAP_CS_getSMCLK()/1000000); //12 Mhz clock -> time in us

                        bool retryDetected = false;
                        uint16_t tmpV = 0;
                        uint16_t retryTime = 0;
                        while(!retryDetected){
                            while(!(ADC_INT1 & status)){
                                status = MAP_ADC14_getEnabledInterruptStatus();
                            }
                            retryTime = MAP_Timer_A_getCounterValue(TIMER_A1_BASE);
                            tmpV = MAP_ADC14_getResult(ADC_MEM1) * 5000 / 16384;
                            if(tmpV > 2500){
                                retryDetected = true;
                                MAP_Timer_A_stopTimer(TIMER_A1_BASE);
                            }else if(retryTime /(MAP_CS_getACLK()/1000) > 5000){
                                serial.print("RETRY TOOK TOO LONG!");
                                retryTime = 0;
                                retryDetected = true;
                                MAP_Timer_A_stopTimer(TIMER_A1_BASE);
                            }
                            MAP_ADC14_clearInterruptFlag(status);
                            status = MAP_ADC14_getEnabledInterruptStatus();
                        }

                        retryTime = retryTime /(MAP_CS_getACLK()/1000); //16.384 kHz -> ms

                        for(int q = 0; q < nrOfMeas; q++){
                            outputVoltages[q] = outputVoltages[q] * 5000 / 16384;
                            serial.print("ShortCircuit OUTPUT: ");
                            serial.print(outputVoltages[q], DEC);
                            serial.println();
                            workingBuffer.getPayload()[2+2*q] = (uint8_t) ((outputVoltages[q] & 0xFF00) >> 8);
                            workingBuffer.getPayload()[2+2*q+1] = (uint8_t) (outputVoltages[q] & 0x00FF);
                        }
                        serial.print("time of measurements: ");
                        serial.print(measTime, DEC);
                        serial.println(" us");
                        workingBuffer.getPayload()[2+2*50] = (uint8_t) ((measTime & 0xFF00) >> 8);
                        workingBuffer.getPayload()[2+2*50+1] = (uint8_t) (measTime & 0x00FF);
                        workingBuffer.setSize(2*50+4);
                        serial.println("Retry Detected!");
                        serial.print("Retry Time: ");
                        serial.print(retryTime, DEC);
                        serial.println(" ms");
                        enableLowCShort(false);
                    }else{
                        serial.println("Short is automatically turned off and Bus needs to be turned on first.");
                    }
                    break;
                case TESTSERVICEHPI_HIGHC_ENABLE:
                    MAP_ADC14_disableConversion();
                    MAP_ADC14_clearInterruptFlag( MAP_ADC14_getEnabledInterruptStatus() );
                    MAP_ADC14_configureMultiSequenceMode(ADC_MEM2, ADC_MEM3, true);
                    MAP_ADC14_enableInterrupt(ADC_INT3);
                    MAP_ADC14_enableConversion();


                    enableHighC(command.getPayload()[3] == 1);
                    this->highCStatus = (command.getPayload()[3] == 1);
                    //wait for voltage Settle
                    d = 0.05 * MAP_CS_getMCLK();
                    for(uint32_t k = 0; k < d;  k++)
                    {
                        __asm("  nop");
                    }
                    //Trigger ADC Conversion!
                    MAP_ADC14_toggleConversionTrigger();

                    //wait for ADC Result
                    status = MAP_ADC14_getEnabledInterruptStatus();
                    while(!(ADC_INT3 & status)){
                        serial.print("waiting!");
                        status = MAP_ADC14_getEnabledInterruptStatus();;
                    }
                    MAP_ADC14_clearInterruptFlag(status);

                    inputVoltage = MAP_ADC14_getResult(ADC_MEM2)* 5000 / 16384;
                    outputVoltage = MAP_ADC14_getResult(ADC_MEM3)* 5000 / 16384;

                    serial.print("INTPUT: ");
                    serial.print(inputVoltage, DEC);
                    serial.print("  |  OUTPUT: ");
                    serial.print(outputVoltage, DEC);
                    serial.println();

                    workingBuffer.getPayload()[2] = (uint8_t) ((inputVoltage & 0xFF00) >> 8);
                    workingBuffer.getPayload()[3] = (uint8_t) (inputVoltage & 0x00FF);
                    workingBuffer.getPayload()[4] = (uint8_t) ((outputVoltage & 0xFF00) >> 8);
                    workingBuffer.getPayload()[5] = (uint8_t) (outputVoltage & 0x00FF);
                    workingBuffer.setSize(6);
                    break;
                case TESTSERVICEHPI_HIGHC_TRIGGERSC:
                    if(command.getPayload()[3] == 1 && highCStatus == true){
                        uint8_t controlTable[16];
                        for(int j = 0; j < 16; j++){
                            controlTable[j] = 0;
                        }
                        //configure Timer
                        timerConfig =
                        {
                                TIMER_A_CLOCKSOURCE_SMCLK,           // SMCLK Clock Source (48Mhz/4)
                                TIMER_A_CLOCKSOURCE_DIVIDER_1,       // SMCLK/1 = 12MHz
                                TIMER_A_TAIE_INTERRUPT_DISABLE,      // Disable Timer ISR
                                TIMER_A_DO_CLEAR                     // Do Clear Counter
                        };
                        slowtimerConfig =
                        {
                                TIMER_A_CLOCKSOURCE_ACLK,           //  ACLK Clock Source (32768 hz)
                                TIMER_A_CLOCKSOURCE_DIVIDER_1,       // ACLK/2 = 16384
                                TIMER_A_TAIE_INTERRUPT_DISABLE,      // Disable Timer ISR
                                TIMER_A_DO_CLEAR                     // Do Clear Counter
                        };
                        MAP_Timer_A_configureContinuousMode(TIMER_A0_BASE, &timerConfig);
                        MAP_Timer_A_configureContinuousMode(TIMER_A1_BASE, &slowtimerConfig);

                        //reconfigure ADC
                        MAP_ADC14_disableConversion();
                        MAP_ADC14_clearInterruptFlag( MAP_ADC14_getEnabledInterruptStatus() );
                        MAP_ADC14_configureSingleSampleMode(ADC_MEM3, true);
                        MAP_ADC14_enableInterrupt(ADC_INT3);
                        MAP_DMA_enableModule();
                        MAP_DMA_setControlBase(controlTable);
                        MAP_DMA_assignChannel(DMA_CH7_ADC14);
                        MAP_DMA_setChannelControl(UDMA_PRI_SELECT | DMA_CH7_ADC14,
                            UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
                        MAP_DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CH7_ADC14,
                            UDMA_MODE_BASIC, (void*) &ADC14->MEM[3],
                            outputVoltages, nrOfMeas);
                        MAP_DMA_assignInterrupt(DMA_INT1, 7);
                        MAP_DMA_registerInterrupt(DMA_INT1, DMA_INT1_IRQHandler);
                        MAP_Interrupt_enableInterrupt(INT_DMA_INT1);

                        DMADone = false;

                        MAP_DMA_enableChannel(7);
                        MAP_ADC14_enableConversion();
                        enableHighCShort(command.getPayload()[3] == 1);
                        MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);
                        MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);
                        MAP_ADC14_toggleConversionTrigger();

                        while(!DMADone);
                        //serial.println("DMA DONE!");

                        uint16_t measTime = MAP_Timer_A_getCounterValue(TIMER_A0_BASE);
                        measTime = measTime/(MAP_CS_getSMCLK()/1000000); //12 Mhz clock -> time in us

                        bool retryDetected = false;
                        uint16_t tmpV = 0;
                        uint16_t retryTime = 0;
                        while(!retryDetected){
                            while(!(ADC_INT3 & status)){
                                status = MAP_ADC14_getEnabledInterruptStatus();
                            }
                            retryTime = MAP_Timer_A_getCounterValue(TIMER_A1_BASE);
                            tmpV = MAP_ADC14_getResult(ADC_MEM3) * 5000 / 16384;
                            if(tmpV > 2500){
                                retryDetected = true;
                                MAP_Timer_A_stopTimer(TIMER_A1_BASE);
                            }else if(retryTime /(MAP_CS_getACLK()/1000) > 5000){
                                serial.println("RETRY TOOK TOO LONG!");
                                retryTime = 0;
                                retryDetected = true;
                                MAP_Timer_A_stopTimer(TIMER_A1_BASE);
                            }
                            MAP_ADC14_clearInterruptFlag(status);
                            status = MAP_ADC14_getEnabledInterruptStatus();
                        }

                        retryTime = retryTime /(MAP_CS_getACLK()/1000); //16.384 kHz -> ms


                        for(int q = 0; q < nrOfMeas; q++){
                            outputVoltages[q] = outputVoltages[q] * 5000 / 16384;
                            serial.print("ShortCircuit OUTPUT: ");
                            serial.print(outputVoltages[q], DEC);
                            serial.println();
                            workingBuffer.getPayload()[2+2*q] = (uint8_t) ((outputVoltages[q] & 0xFF00) >> 8);
                            workingBuffer.getPayload()[2+2*q+1] = (uint8_t) (outputVoltages[q] & 0x00FF);
                        }
                        serial.print("time of measurements: ");
                        serial.print(measTime, DEC);
                        serial.println(" us");
                        workingBuffer.getPayload()[2+2*50] = (uint8_t) ((measTime & 0xFF00) >> 8);
                        workingBuffer.getPayload()[2+2*50+1] = (uint8_t) (measTime & 0x00FF);
                        workingBuffer.setSize(2*50+4);
                        serial.println("Retry Detected!");
                        serial.print("Retry Time: ");
                        serial.print(retryTime, DEC);
                        serial.println(" ms");
                        enableHighCShort(false);
                    }else{
                        serial.println("Short is automatically turned off and Bus needs to be turned on first.");
                    }
                    break;
            }
        }

        return true;
    }
    return false;
}

