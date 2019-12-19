#include "HPI.h"

// CDHS bus handler
PQ9Bus pq9bus(1, GPIO_PORT_P2, GPIO_PIN1);
//PQ9Bus rs485(3, GPIO_PORT_P9, GPIO_PIN0);

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
PQ9CommandHandler cmdHandler(pq9bus, services, 2);
PeriodicTask timerTask(FCLOCK, periodicTask);
Task* tasks[] = { &cmdHandler, &timerTask };

// system uptime
unsigned long uptime = 0;

// TODO: remove when bug in CCS has been solved
void validCmd(PQ9Frame &newFrame)
{
    cmdHandler.received(newFrame);
}

void periodicTask()
{
    signed short i, t;
    unsigned short v;

    // increase the timer, this happens every second
    uptime++;


    unsigned long storedUptime;
    fram.read(0, (unsigned char *)&storedUptime, sizeof(storedUptime));

    serial.print("Stored uptime: ");
    serial.print(storedUptime, DEC);
    serial.println();

    serial.print("Uptime: ");
    serial.print(uptime, DEC);
    serial.println();

    if (uptime >= 15)
    {
        serial.println("try reboot");
        reset.forceHardReset();
    }


        PQ9Frame frame;
        //frame.setDestination(1);
        //frame.setSource(2);
        frame.setDestination(2);
        frame.setSource(1);
        frame.setPayloadSize(2);
        frame.getPayload()[0] = 17; // ping
        frame.getPayload()[1] = 1;    // request
        //rs485.transmit(frame);
        //pq9bus.transmit(frame);

    // collect telemetry
    serial.print("Temperature: ");
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
    serial.println(" mA");

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
    pq9bus.begin(115200, 100);    // baud rate: 115200 bps
                                            // address ?

    //rs485.begin(115200, 2);    // baud rate: 115200 bps
                                                // address ?

    // link the command handler to the PQ9 bus:
    // every time a new command is received, it will be forwarded to the command handler
    // TODO: put back the lambda function after bug in CCS has been fixed
    //pq9bus.setReceiveHandler([](PQ9Frame &newFrame){ cmdHandler.received(newFrame); });
    pq9bus.setReceiveHandler(validCmd);
    //rs485.setReceiveHandler(validCmd);

    // every time a command is correctly processed, call the watch-dog
    // TODO: put back the lambda function after bug in CCS has been fixed
    //cmdHandler.onValidCommand([]{ reset.kickInternalWatchDog(); });
    //cmdHandler.onValidCommand(validCmd);

    serial.println("HPI booting...");

    TaskManager::start(tasks, 2);
}
