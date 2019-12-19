#include "HPI.h"

// CDHS bus handler
PQ9Bus pq9bus(3, GPIO_PORT_P9, GPIO_PIN0);

// debug console handler
DSerial serial;

// I2C bus
DWire i2c(0);
TMP100 tempSensor(i2c, 0x48);
INA226 powerBus(i2c, 0x40);

// SPI bus
DSPI spi(3);

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
void kickWatchdog(PQ9Frame &newFrame)
{
    cmdHandler.received(newFrame);
}

void periodicTask()
{
    signed short i, t;
    unsigned short v;

    // increase the timer, this happens every second
    uptime++;

    serial.print("Uptime: ");
    serial.print(uptime, DEC);
    serial.println();

    if (uptime >= 5)
    {
        serial.println("try reboot");
        reset.forceReset();
    }

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

    serial.begin( );                        // baud rate: 9600 bps
    pq9bus.begin(115200, COMMS_ADDRESS);    // baud rate: 115200 bps
                                            // address COMMS (4)

    // link the command handler to the PQ9 bus:
    // every time a new command is received, it will be forwarded to the command handler
    // TODO: put back the lambda function after bug in CCS has been fixed
    //pq9bus.setReceiveHandler([](PQ9Frame &newFrame){ cmdHandler.received(newFrame); });
    pq9bus.setReceiveHandler(kickWatchdog);

    // every time a command is correctly processed, call the watch-dog
    // TODO: put back the lambda function after bug in CCS has been fixed
    //cmdHandler.onValidCommand([]{ reset.kickInternalWatchDog(); });
    //cmdHandler.onValidCommand(validCmd);

    serial.println("HPI booting...");

    TaskManager::start(tasks, 2);
}
