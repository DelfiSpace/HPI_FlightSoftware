#include "HPI.h"

// CDHS bus handler
PQ9Bus pq9bus(1, GPIO_PORT_P2, GPIO_PIN1);
RS485 rs485(3, GPIO_PORT_P9, GPIO_PIN0);

// I2C bus
DWire i2c(0);
TMP100 tempSensor(i2c, 0x48);
INA226 powerBus(i2c, 0x40);

// SPI bus
DSPI spi(3);
MB85RS fram(spi, GPIO_PORT_P1, GPIO_PIN0 );

// HardwareMonitor
HWMonitor hwMonitor(&fram);

// services running in the system
PingService ping;
ResetService reset( GPIO_PORT_P4, GPIO_PIN0 );

Service* services[] = { &ping, &reset };

// HPI board tasks
CommandHandler<PQ9Frame,PQ9Message> cmdHandler(rs485, services, 2);
PeriodicTask timerTask(1000, periodicTask);
PeriodicTask* periodicTasks[] = {&timerTask};
PeriodicTaskNotifier taskNotifier = PeriodicTaskNotifier(periodicTasks, 1);
Task* tasks[] = { &timerTask, &cmdHandler };

// used to forward PQ9Frames
volatile unsigned char sourceAddress = HPI_ADDRESS;

// system uptime
unsigned long uptime = 0;

PQ9Frame pingCmd, bus2On, bus2Off;

// TODO: remove when bug in CCS has been solved
void newPQ9Cmd(DataFrame &newFrame)
{
    //serial.println("validPQ9Cmd");
    newFrame.setDestination(sourceAddress);
    rs485.transmit(newFrame);
}

void newRS485Cmd(DataFrame &newFrame)
{
    //serial.println("validRS485Cmd");
    if (newFrame.getDestination() == HPI_ADDRESS)
    {
        //serial.println("Mine!");
        cmdHandler.received(newFrame);
    }
    else
    {
        // forward command to PQ9 bus
        sourceAddress = newFrame.getSource();
        newFrame.setSource( HPI_ADDRESS );
        pq9bus.transmit(newFrame);
    }
}

void validRS485Cmd( void )
{
    reset.kickInternalWatchDog();
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

    // initialize the ADC
    // - ADC14 and FPU Module
    // - MEM0 for internal temperature measurements
    ADCManager::initADC();

    // init I2c bus and I2C devices
    i2c.setFastMode();
    i2c.begin();
    tempSensor.init();
    powerBus.setShuntResistor(40);

    // init SPI bus
    spi.initMaster(DSPI::MODE0, DSPI::MSBFirst, 1000000);
    fram.init();

    Console::init( 115200 );                              // baud rate: 9600 bps
    pq9bus.begin(115200, HPI_ADDRESS);              // baud rate: 115200 bps
                                                    // address 100

    rs485.init(9600, HPI_ADDRESS);                  // baud rate: 9600 bps

    // initialize the reset handler:
    // - prepare the watch-dog
    // - initialize the pins for the hardware watch-dog
    // - prepare the pin for power cycling the system
    reset.init();

    // initialize Task Notifier
    taskNotifier.init();

    // initialize HWMonitor readings
    hwMonitor.readResetStatus();
    hwMonitor.readCSStatus();

    // link the command handler to the PQ9 bus:
    // every time a new command is received, it will be forwarded to the command handler
    // TODO: put back the lambda function after bug in CCS has been fixed
    pq9bus.setReceiveHandler(newPQ9Cmd);
    rs485.setReceiveHandler(newRS485Cmd);

    // every time a command is correctly processed, call the watch-dog
    // TODO: put back the lambda function after bug in CCS has been fixed
    //cmdHandler.onValidCommand([]{ reset.kickInternalWatchDog(); });
    cmdHandler.onValidCommand(validRS485Cmd);

    Console::log("HPI booting...");

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
