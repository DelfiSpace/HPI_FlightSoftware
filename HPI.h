/*
 * HPI.h
 *
 *  Created on: 23 Jul 2019
 *      Author: stefanosperett
 */

#ifndef HPI_H_
#define HPI_H_

#include <driverlib.h>
#include "msp.h"
#include "CommandHandler.h"
#include "DelfiPQcore.h"
#include "PQ9Bus.h"
#include "RS485.h"
#include "PQ9Frame.h"
#include "DWire.h"
#include "TMP100.h"
#include "INA226.h"
#include "DSPI.h"
#include "MB85RS.h"
#include "DSerial.h"
#include "Task.h"
#include "PeriodicTask.h"
#include "TaskManager.h"
#include "Service.h"
#include "PingService.h"
#include "ResetService.h"
#include "PeriodicTaskNotifier.h"

#define FCLOCK                  48000000
#define HPI_ADDRESS             100

// callback functions
void periodicTask();
void validCmd(PQ9Frame &frame);

#endif /* HPI_H_ */
