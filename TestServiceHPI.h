/*
 * TestService.h
 *
 *  Created on: 13 Feb 2020
 *      Author: Casper
 */

#ifndef TESTSERVICEHPI_H_
#define TESTSERVICEHPI_H_

#include "Service.h"
#include "DSerial.h"

#define TESTSERVICEHPI_NR              0
#define TESTSERVICEHPI_LOWC_ENABLE     1
#define TESTSERVICEHPI_LOWC_TRIGGERSC  2
#define TESTSERVICEHPI_HIGHC_ENABLE     3
#define TESTSERVICEHPI_HIGHC_TRIGGERSC  4


class TestServiceHPI: public Service
{
private:
    lowCStatus = false;
    highCStatus = false;

 public:
    TestServiceHPI();
     virtual bool process( DataMessage &command, DataMessage &workingBbuffer );
};



#endif /* TESTSERVICE_H_ */
