/*
 * COMMS.h
 *
 *  Created on: 23 Jul 2019
 *      Author: stefanosperett
 */

#ifndef EPS_H_
#define EPS_H_

#include <driverlib.h>
#include "msp.h"
#include "CommandHandler.h"
#include "DelfiPQcore.h"
#include "PQ9Bus.h"
#include "PQ9Frame.h"
#include "DWire.h"
#include "TMP100.h"
#include "INA226.h"
#include "DSPI.h"
#include "DSerial.h"
#include "Task.h"
#include "PeriodicTask.h"
#include "TaskManager.h"
#include "Service.h"
#include "PingService.h"
#include "ResetService.h"

#define FCLOCK 48000000

#define COMMS_ADDRESS     4

// callback functions
void periodicTask();

#endif /* COMMS_H_ */
