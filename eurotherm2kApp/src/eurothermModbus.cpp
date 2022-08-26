/* eurothermModbus.c
 * This file lists the driver routines used to communicate with
 * a Eurotherm device using the ModBus over serial interface
 * This package depends on the asyn and modbus EPICS packages
 * to communicate with the device
 *
 * Copyright (c) 2009 Stanford University as operator of 
 *     SLAC National Accelerator Laboratory
 *
 * Modified 15/04/2011 for use at DLS
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <epicsStdio.h>
#include <iocsh.h>
#include <epicsExport.h>
#include "registryFunction.h"
#include <asynDriver.h>
#include <asynOctetSyncIO.h>
#include <asynCommonSyncIO.h>
#include <asynStandardInterfaces.h>
#include <drvModbusAsyn.h>
#include <errlog.h>

#define EUROTHERM_MAX_TIMEOUT_MS	1000
#define EUROTHERM_POLL_MS		1000
#define DRIVER_NAME			(__FILE__)
#define LOG_ERROR(str...)		do { \
						errlogPrintf("%s::%s(@%d):", DRIVER_NAME, __func__, __LINE__); \
						errlogPrintf(str); \
						errlogPrintf("\n"); \
					} while(0)

/* Modbus function codes */
#define MODBUS_READW	3
#define MODBUS_WRITEW	6

/* Modbus address space definitions for loop parameters */
#define LOOP1_STARTADDR 1
#define LOOP2_STARTADDR 1025
#define LOOP3_STARTADDR 2049
#define PV_OFFSET		0 		/*Process variable, setpoint and target output (3 locations)*/
#define RR_OFFSET		34		/*Ramp rate*/
#define MAN_OFFSET 		272		/*Auto/manual select*/
#define PID_OFFSET		350		/*PID parameters (3 locations)*/

/* Modbus address space definitions for controller parameters */
#define AUTOTUNE_ADDR	3072	/*Autotune parameters (6 locations)*/


/* Create modbus port name in the form:
	Eurotherm_ASYNPORT_ADDR_LOOPNO_Rd_VarName 
	Eurotherm_ASYNPORT_ADDR_LOOPNO_Wr_VarName 
	Controller variables use loop 0
*/
int mkname(char *str, int max, char *asynPortName, int modbusAddress, int loopNumber, int Write, char *szVariable)
{
	int ret;
	
	if(str == NULL)
		return strlen("Eurotherm__XX_XX_XX_") + strlen(asynPortName) + strlen(szVariable);
	ret = sprintf(str, "Eurotherm_%s_%d_%d_%s_%s", asynPortName, modbusAddress, loopNumber, Write? "Wr" : "Rd", szVariable);
	if(ret >= 0) ret++;	/* Add the terminating char */
	return ret;
}


/* Configures modbus ports for general controller parameters */
static int eurothermModbusCtrlConfigure(char *asynPortName, int modbusAddress)
{
	asynStatus asynRet;
	char *str;
	int strmaxlen;
	
	if (asynPortName == NULL) {
		LOG_ERROR("Invalid asynPortName argument (NULL)");
		return -EINVAL;
	}
	if ((modbusAddress < 0) || (modbusAddress >= 100)) {
		LOG_ERROR("Invalid modbusAddress value %d (0<=v<100)", modbusAddress);
		return -EINVAL;
	}
	
	strmaxlen = mkname(NULL, 0, asynPortName, modbusAddress, 0, 0, "MaxNameSize");
	if (strmaxlen < 0) {
		LOG_ERROR("Could not generate modbus names: %s", strerror(errno));
        // no need to call free(str) here as it has not yet been malloc'd
		//free(str);
		return -errno;
	}
	str = malloc(strmaxlen);
	if(str == NULL) {
		LOG_ERROR("Could not allocate %d bytes of memory", strmaxlen);
		free(str);
		return -ENOMEM;
	}
	
	/* Create modbus channels for the autotune parameters */
	mkname(str, strmaxlen, asynPortName, modbusAddress, 0, 0, "ATUNE");
	asynRet = drvModbusAsynConfigure(str, asynPortName, modbusAddress,
				MODBUS_READW, AUTOTUNE_ADDR, 6,
				0, EUROTHERM_POLL_MS, "");
	if (asynRet != asynSuccess) {
		LOG_ERROR("drvModbusAsynConfigure failed with error %d", asynRet);
		free(str);
		return -EIO;
	}
	mkname(str, strmaxlen, asynPortName, modbusAddress, 0, 1, "ATUNE");
	asynRet = drvModbusAsynConfigure(str, asynPortName, modbusAddress,
				MODBUS_WRITEW, AUTOTUNE_ADDR, 6,
				0, EUROTHERM_POLL_MS, "");
	if (asynRet != asynSuccess) {
		LOG_ERROR("drvModbusAsynConfigure failed with error %d", asynRet);
		free(str);
		return -EIO;
	}
	
	free(str);
	return 0;
}

/* Configures modbus ports for a specified loop on a Eurotherm 2000 series controller */
static int eurothermModbusLoopConfigure(char *asynPortName, int modbusAddress, int loopNumber)
{
	asynStatus asynRet;
	char *str;
	int strmaxlen;
	int loopStart = 0;

	if (asynPortName == NULL) {
		LOG_ERROR("Invalid asynPortName argument (NULL)");
		return -EINVAL;
	}
	if ((modbusAddress < 0) || (modbusAddress >= 100)) {
		LOG_ERROR("Invalid modbusAddress value %d (0<=v<100)", modbusAddress);
		return -EINVAL;
	}
	if ((loopNumber < 1) || (loopNumber>3)) {
		LOG_ERROR("Invalid loopNumber value %d (1<=v<=3)", loopNumber);
		return -EINVAL;
	}
	
	switch(loopNumber) {
		case 1:
			loopStart=LOOP1_STARTADDR;
		break;
		case 2:
			loopStart=LOOP2_STARTADDR;
		break;
		case 3:
			loopStart=LOOP3_STARTADDR;
		break;
	}
	
	strmaxlen = mkname(NULL, 0, asynPortName, modbusAddress, loopNumber, 0, "MaxNameSize");
	if (strmaxlen < 0) {
		LOG_ERROR("Could not generate modbus names: %s", strerror(errno));
        // no need to call free(str) here as it has not yet been malloc'd
		//free(str);
		return -errno;
	}
	str = malloc(strmaxlen);
	if(str == NULL) {
		LOG_ERROR("Could not allocate %d bytes of memory", strmaxlen);
		free(str);
		return -ENOMEM;
	}
	
	/* Create modbus channels for the PV, target setpoint and manual output */
	mkname(str, strmaxlen, asynPortName, modbusAddress, loopNumber, 0, "PV");
	asynRet = drvModbusAsynConfigure(str, asynPortName, modbusAddress,
				MODBUS_READW, loopStart+PV_OFFSET, 3,
				0, EUROTHERM_POLL_MS, "");
	if (asynRet != asynSuccess) {
		LOG_ERROR("drvModbusAsynConfigure failed with error %d", asynRet);
		free(str);
		return -EIO;
	}
	mkname(str, strmaxlen, asynPortName, modbusAddress, loopNumber, 1, "PV");
	asynRet = drvModbusAsynConfigure(str, asynPortName, modbusAddress,
				MODBUS_WRITEW, loopStart+PV_OFFSET, 3,
				0, EUROTHERM_POLL_MS, "");
	if (asynRet != asynSuccess) {
		LOG_ERROR("drvModbusAsynConfigure failed with error %d", asynRet);
		free(str);
		return -EIO;
	}
	
	/* Create modbus channels for the ramp rate */
	mkname(str, strmaxlen, asynPortName, modbusAddress, loopNumber, 0, "RR");
	asynRet = drvModbusAsynConfigure(str, asynPortName, modbusAddress,
				MODBUS_READW, loopStart+RR_OFFSET, 1,
				0, EUROTHERM_POLL_MS, "");
	if (asynRet != asynSuccess) {
		LOG_ERROR("drvModbusAsynConfigure failed with error %d", asynRet);
		free(str);
		return -EIO;
	}
	mkname(str, strmaxlen, asynPortName, modbusAddress, loopNumber, 1, "RR");
	asynRet = drvModbusAsynConfigure(str, asynPortName, modbusAddress,
				MODBUS_WRITEW, loopStart+RR_OFFSET, 1,
				0, EUROTHERM_POLL_MS, "");
	if (asynRet != asynSuccess) {
		LOG_ERROR("drvModbusAsynConfigure failed with error %d", asynRet);
		free(str);
		return -EIO;
	}
	
	/* Create modbus channels for the manual/auto mode */
	mkname(str, strmaxlen, asynPortName, modbusAddress, loopNumber, 0, "MAN");
	asynRet = drvModbusAsynConfigure(str, asynPortName, modbusAddress,
				MODBUS_READW, loopStart+MAN_OFFSET, 1,
				0, EUROTHERM_POLL_MS, "");
	if (asynRet != asynSuccess) {
		LOG_ERROR("drvModbusAsynConfigure failed with error %d", asynRet);
		free(str);
		return -EIO;
	}
	mkname(str, strmaxlen, asynPortName, modbusAddress, loopNumber, 1, "MAN");
	asynRet = drvModbusAsynConfigure(str, asynPortName, modbusAddress,
				MODBUS_WRITEW, loopStart+MAN_OFFSET, 1,
				0, EUROTHERM_POLL_MS, "");
	if (asynRet != asynSuccess) {
		LOG_ERROR("drvModbusAsynConfigure failed with error %d", asynRet);
		free(str);
		return -EIO;
	}
	
	/* Create modbus channels for the PID parameters */
	mkname(str, strmaxlen, asynPortName, modbusAddress, loopNumber, 0, "PID");
	asynRet = drvModbusAsynConfigure(str, asynPortName, modbusAddress,
				MODBUS_READW, loopStart+PID_OFFSET, 3,
				0, EUROTHERM_POLL_MS, "");
	if (asynRet != asynSuccess) {
		LOG_ERROR("drvModbusAsynConfigure failed with error %d", asynRet);
		free(str);
		return -EIO;
	}
	mkname(str, strmaxlen, asynPortName, modbusAddress, loopNumber, 1, "PID");
	asynRet = drvModbusAsynConfigure(str, asynPortName, modbusAddress,
				MODBUS_WRITEW, loopStart+PID_OFFSET, 3,
				0, EUROTHERM_POLL_MS, "");
	if (asynRet != asynSuccess) {
		LOG_ERROR("drvModbusAsynConfigure failed with error %d", asynRet);
		free(str);
		return -EIO;
	}
	
	free(str);
	return 0;
}

/**EPICS registration API
 * We provide this registrar to make it easy to add a Eurotherm device without
 * having to mess up st.cmd
 */
 
static const iocshArg ConfigureArg0 = {"Asyn Port Name", iocshArgString};
static const iocshArg ConfigureArg1 = {"Modbus Slave Address", iocshArgInt};
static const iocshArg ConfigureArg2 = {"Loop Number", iocshArgInt};

static const iocshArg * const eurothermModbusCtrlConfigureArgs[2] = {
	&ConfigureArg0,
	&ConfigureArg1,
};

static const iocshArg * const eurothermModbusLoopConfigureArgs[3] = {
	&ConfigureArg0,
	&ConfigureArg1,
	&ConfigureArg2,
};

static const iocshFuncDef eurothermModbusCtrlConfigureFuncDef=
                                                    {"eurothermModbusCtrlConfigure", 2,
                                                     eurothermModbusCtrlConfigureArgs};

static const iocshFuncDef eurothermModbusLoopConfigureFuncDef=
                                                    {"eurothermModbusLoopConfigure", 3,
                                                     eurothermModbusLoopConfigureArgs};                                             

static void eurothermModbusCtrlConfigureCall(const iocshArgBuf *args)
{
	int ret;

	ret = eurothermModbusCtrlConfigure(args[0].sval, args[1].ival);

	if(ret != 0)
		printf("Eurotherm modbus controller init failed with code %d.\n", ret);
}

static void eurothermModbusLoopConfigureCall(const iocshArgBuf *args)
{
	int ret;

	ret = eurothermModbusLoopConfigure(args[0].sval, args[1].ival, args[2].ival);

	if(ret != 0)
		printf("Eurotherm modbus loop init failed with code %d.\n", ret);
}

static void eurothermModbusRegister() {
	iocshRegister(&eurothermModbusCtrlConfigureFuncDef, eurothermModbusCtrlConfigureCall);
	iocshRegister(&eurothermModbusLoopConfigureFuncDef, eurothermModbusLoopConfigureCall);
}

epicsExportRegistrar(eurothermModbusRegister);


