#include "HPI.h"

// CDHS bus handler
//PQ9Bus pq9bus(1, GPIO_PORT_P2, GPIO_PIN1);
RS485 rs485(3, GPIO_PORT_P9, GPIO_PIN0);


// debug console handler
DSerial serial;

// I2C bus
DWire i2c(0);
TMP100 tempSensor(i2c, 0x48);
INA226 powerBus(i2c, 0x40);

// SPI bus
DSPI spi(3);
MB85RS fram(spi, GPIO_PORT_P1, GPIO_PIN0 );

// services running in the system
PingService ping;
ResetService reset( GPIO_PORT_P4, GPIO_PIN0 );

Service* services[] = { &ping, &reset };

// COMMS board tasks
//PQ9CommandHandler cmdHandler(pq9bus, services, 2);
PeriodicTask timerTask(FCLOCK, periodicTask);
Task* tasks[] = { &timerTask };

// system uptime
unsigned long uptime = 0;

PQ9Frame pingCmd, bus2On, bus2Off;

// TODO: remove when bug in CCS has been solved
void validPQ9Cmd(PQ9Frame &newFrame)
{
    serial.println("validPQ9Cmd");
    rs485.transmit(newFrame);
}

void validRS485Cmd(PQ9Frame &newFrame)
{
    serial.println("validRS485Cmd");
    if (newFrame.getDestination() == 100)
    {
        serial.println("Mine!");
        newFrame.setDestination(1);
        newFrame.setSource(100);
        newFrame.getPayload()[1] = 2;
        rs485.transmit(newFrame);
        //cmdHandler.received(newFrame);
    }
    else
    {
        // forward command to PQ9 bus
        //pq9bus.transmit(newFrame);
    }
}

void periodicTask()
{
    signed short i, t;
    unsigned short v;

    // increase the timer, this happens every second
    uptime++;


    unsigned long storedUptime;
    fram.read(0, (unsigned char *)&storedUptime, sizeof(storedUptime));

    /*serial.print("Stored uptime: ");
    serial.print(storedUptime, DEC);
    serial.println();

    serial.print("Uptime: ");
    serial.print(uptime, DEC);
    serial.println();*/

/*
    if ((uptime & 0x0F) == 0)
    {
        //for (int h = 0; h < 1000; h++)
        rs485.transmit(pingCmd);
    }

    if ((uptime & 0x0F) == 2)
    {
        //for (int h = 0; h < 1000; h++)
        rs485.transmit(bus2On);
    }

    if ((uptime & 0x0F) == 7)
    {
        //for (int h = 0; h < 100; h++)
        rs485.transmit(bus2Off);
    }
*/
    // collect telemetry
    /*serial.print("Temperature: ");
    tempSensor.getTemperature(t);
    serial.print(t, DEC);
    serial.println("00 mdegC");

    // collect telemetry
    serial.print("Voltage: ");
    powerBus.getVoltage(v);
    serial.print(v, DEC);
    serial.println(" mV");

    serial.print("Current: ");
    powerBus.getCurrent(i);
    serial.print(i, DEC);
    serial.println(" mA");*/

    // refresh the watch-dog configuration to make sure that, even in case of internal
    // registers corruption, the watch-dog is capable of recovering from an error
    //reset.refreshConfiguration();

    // kick hardware watch-dog after every telemetry collection happens
    reset.kickExternalWatchDog();

    // TODO: check if this is the only option
    // kick the internal watchdog as well
    reset.kickInternalWatchDog();

    fram.write(0, (unsigned char *)&uptime, sizeof(uptime));
}

/**
 * main.cpp
 */
void main(void)
{
    // initialize the MCU:
    // - clock source
    // - clock tree
    DelfiPQcore::initMCU();

    // initialize the reset handler:
    // - prepare the watch-dog
    // - initialize the pins for the hardware watch-dog
    // - prepare the pin for power cycling the system
    reset.init();

    // init I2c bus and I2C devices
    i2c.setFastMode();
    i2c.begin();
    tempSensor.init();
    powerBus.setShuntResistor(40);

    // init SPI bus
    spi.initMaster(DSPI::MODE0, DSPI::MSBFirst, 1000000);
    fram.init();

    serial.begin( );                        // baud rate: 9600 bps
    //pq9bus.begin(115200, 100);              // baud rate: 115200 bps
                                            // address 100
//todo: set 1200
    rs485.init(1200);                     // baud rate: 9600 bps


    // link the command handler to the PQ9 bus:
    // every time a new command is received, it will be forwarded to the command handler
    // TODO: put back the lambda function after bug in CCS has been fixed
    //pq9bus.setReceiveHandler(validPQ9Cmd);
    rs485.setReceptionHandler(validRS485Cmd);

    // every time a command is correctly processed, call the watch-dog
    // TODO: put back the lambda function after bug in CCS has been fixed
    //cmdHandler.onValidCommand([]{ reset.kickInternalWatchDog(); });
    //cmdHandler.onValidCommand(validCmd);

    serial.println("HPI booting...");

    // ping request
    pingCmd.setDestination(2);
    pingCmd.setSource(100);
    pingCmd.setPayloadSize(2);
    pingCmd.getPayload()[0] = 17;
    pingCmd.getPayload()[1] = 1;

    // bus 2 ON
    bus2On.setDestination(2);
    bus2On.setSource(100);
    bus2On.setPayloadSize(4);
    bus2On.getPayload()[0] = 1;
    bus2On.getPayload()[1] = 1;
    bus2On.getPayload()[2] = 2;
    bus2On.getPayload()[3] = 1;

    // bus 2 OFF
    bus2Off.setDestination(2);
    bus2Off.setSource(100);
    bus2Off.setPayloadSize(4);
    bus2Off.getPayload()[0] = 1;
    bus2Off.getPayload()[1] = 1;
    bus2Off.getPayload()[2] = 2;
    bus2Off.getPayload()[3] = 0;

    TaskManager::start(tasks, 2);
}
