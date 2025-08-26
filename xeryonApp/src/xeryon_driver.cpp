#include <asynOctetSyncIO.h>
#include <cstdio>
#include <cstdlib>
#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <iostream>

#include "xeryon_driver.hpp"

constexpr int NUM_PARAMS = 0;

// actual encoder res is ??? microdegrees
// MRES -> EGU
// 1.0  -> microdegrees
// 1e-3 -> millidegrees
// 1e-6 -> degrees
// constexpr double DRIVER_RESOLUTION = ?;

XeryonMotorController::XeryonMotorController(const char *portName, const char *XeryonMotorPortName,
                                           int numAxes, double movingPollPeriod,
                                           double idlePollPeriod)
    : asynMotorController(portName, numAxes, NUM_PARAMS,
                          0, // No additional interfaces beyond the base class
                          0, // No additional callback interfaces beyond those in base class
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                          1,    // autoconnect
                          0, 0) // Default priority and stack size
{
    asynStatus status;
    int axis;
    XeryonMotorAxis *pAxis;
    static const char *functionName = "XeryonMotorController::XeryonMotorController";

    // Connect to motor controller
    status = pasynOctetSyncIO->connect(XeryonMotorPortName, 0, &pasynUserController_, NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to Xeryon controller\n", functionName);
    }

    // Create XeryonMotorAxis object for each axis
    // if not done here, user must call XeryonMotorCreateAxis from cmd file
    for (axis = 0; axis < numAxes; axis++) {
        pAxis = new XeryonMotorAxis(this, axis);
    }

    // to avoid compiler warning, maybe just don't create pAxis variable at all?
    (void)pAxis;

    startPoller(movingPollPeriod, idlePollPeriod, 0);
}

extern "C" int XeryonMotorCreateController(const char *portName, const char *XeryonMotorPortName,
                                          int numAxes, int movingPollPeriod, int idlePollPeriod) {
    XeryonMotorController *pXeryonMotorController =
        new XeryonMotorController(portName,
                                  XeryonMotorPortName,
                                  numAxes,
                                  movingPollPeriod / 1000.,

                                  idlePollPeriod / 1000.);
    (void)pXeryonMotorController;
    return (asynSuccess);
}

void XeryonMotorController::report(FILE *fp, int level) {
    // "dbior" from iocsh can be useful to see what's going on here
    fprintf(fp, "Xeryon Motor Controller driver %s\n", this->portName);
    fprintf(fp, "    numAxes=%d\n", numAxes_);
    fprintf(fp, "    moving poll period=%f\n", movingPollPeriod_);
    fprintf(fp, "    idle poll period=%f\n", idlePollPeriod_);

    // Call the base class method
    asynMotorController::report(fp, level);
}

XeryonMotorAxis *XeryonMotorController::getAxis(asynUser *pasynUser) {
    return static_cast<XeryonMotorAxis *>(asynMotorController::getAxis(pasynUser));
}

XeryonMotorAxis *XeryonMotorController::getAxis(int axisNo) {
    return static_cast<XeryonMotorAxis *>(asynMotorController::getAxis(axisNo));
}

XeryonMotorAxis::XeryonMotorAxis(XeryonMotorController *pC, int axisNo)
    : asynMotorAxis(pC, axisNo), pC_(pC) {

    axisIndex_ = axisNo + 1;
    asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "XeryonMotorAxis created with axis index %d\n", axisIndex_);

    // Gain Support is required for setClosedLoop to be called
    setIntegerParam(pC->motorStatusHasEncoder_, 1);
    setIntegerParam(pC->motorStatusGainSupport_, 1);

    callParamCallbacks();
}

void XeryonMotorAxis::report(FILE *fp, int level) {
    if (level > 0) {
        fprintf(fp, " Axis #%d\n", axisNo_);
        fprintf(fp, " axisIndex_=%d\n", axisIndex_);
    }
    asynMotorAxis::report(fp, level);
}

asynStatus XeryonMotorAxis::stop(double acceleration) {
    asynStatus asyn_status = asynSuccess;

    sprintf(pC_->outString_, "STOP");
    asyn_status = pC_->writeReadController();

    callParamCallbacks();
    return asyn_status;
}

asynStatus XeryonMotorAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration) {
    asynStatus asyn_status = asynSuccess;

    std::ostringstream oss;
    oss << "DPOS=" << std::to_string(position);
    sprintf(pC_->outString_, "%s", oss.str().c_str());
    asyn_status = pC_->writeReadController();

    callParamCallbacks();
    return asyn_status;
}


asynStatus XeryonMotorAxis::poll(bool *moving) {
    asynStatus asyn_status = asynSuccess;

    // encoder position
    sprintf(pC_->outString_, "EPOS=?");
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        goto skip;
    }
    std::cout << "EPOS = " << pC_->inString_ << "\n";

    // Status bits 0-21
    // 5: Motor on
    // 6: Closed loop
    // 10: Position reached
    // 14: Left end stop
    // 15: Right end stop
    sprintf(pC_->outString_, "STAT");
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        goto skip;
    }
    std::cout << "STAT = " << pC_->inString_ << "\n";

skip:
    callParamCallbacks();
    return asyn_status;
}

asynStatus XeryonMotorAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards) {
    asynStatus asyn_status = asynSuccess;
    callParamCallbacks();
    return asyn_status;
}

asynStatus XeryonMotorAxis::setClosedLoop(bool closedLoop) {
    asynStatus asyn_status = asynSuccess;
    callParamCallbacks();
    return asyn_status;
}

// ==================
// iosch registration
// ==================

static const iocshArg XeryonMotorCreateControllerArg0 = {"asyn port name", iocshArgString};
static const iocshArg XeryonMotorCreateControllerArg1 = {"Controller port name", iocshArgString};
static const iocshArg XeryonMotorCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg XeryonMotorCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg XeryonMotorCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg *const XeryonMotorCreateControllerArgs[] = {
    &XeryonMotorCreateControllerArg0, &XeryonMotorCreateControllerArg1,
    &XeryonMotorCreateControllerArg2, &XeryonMotorCreateControllerArg3,
    &XeryonMotorCreateControllerArg4};
static const iocshFuncDef XeryonMotorCreateControllerDef = {"XeryonMotorCreateController", 5,
                                                           XeryonMotorCreateControllerArgs};

static void XeryonMotorCreateControllerCallFunc(const iocshArgBuf *args) {
    XeryonMotorCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival,
                               args[4].ival);
}

static void XeryonMotorRegister(void) {
    iocshRegister(&XeryonMotorCreateControllerDef, XeryonMotorCreateControllerCallFunc);
}

extern "C" {
epicsExportRegistrar(XeryonMotorRegister);
}
