#include <asynOctetSyncIO.h>
#include <cstdio>
#include <cstdlib>
#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <iostream>
#include <optional>

#include "xeryon_driver.hpp"

constexpr double DRIVER_RESOLUTION = 0.00625; // deg/count

// TODO
constexpr int NUM_PARAMS = 0;

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
    static const char *functionName = "XeryonMotorController::XeryonMotorController";

    // Connect to motor controller
    status = pasynOctetSyncIO->connect(XeryonMotorPortName, 0, &pasynUserController_, NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s: cannot connect to Xeryon controller\n", functionName);
    }

    // Create XeryonMotorAxis object for each axis
    for (axis = 0; axis < numAxes; axis++) {
        new XeryonMotorAxis(this, axis);
    }

    // Set INFO=0 to avoid controller from sending stuff unprompted
    sprintf(this->outString_, "INFO=0");
    writeController();

    // Note: stage type can be made configurable if we want to use this with other stages
    // configure driver for XRTA rotation stage
    sprintf(this->outString_, "XRTA=109");
    writeController();

    startPoller(movingPollPeriod, idlePollPeriod, 0);
}

extern "C" int XeryonMotorCreateController(const char *portName, const char *XeryonMotorPortName,
                                           int numAxes, int movingPollPeriod, int idlePollPeriod) {
    new XeryonMotorController(portName, XeryonMotorPortName, numAxes, movingPollPeriod / 1000.,
                              idlePollPeriod / 1000.);
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
    asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "XeryonMotorAxis created with axis index %d\n",
              axisIndex_);

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
    std::cout << "stop called" << std::endl;
    asyn_status = pC_->writeController();

    callParamCallbacks();
    return asyn_status;
}

asynStatus XeryonMotorAxis::move(double position, int relative, double minVelocity,
                                 double maxVelocity, double acceleration) {
    asynStatus asyn_status = asynSuccess;

    // set the speed which is given as an integer with units 0.01deg/sec
    const int velo = maxVelocity * DRIVER_RESOLUTION * 100;
    sprintf(pC_->outString_, "SSPD=%d", velo);
    asyn_status = pC_->writeController();

    // Move to this target position in closed loop
    sprintf(pC_->outString_, "DPOS=%d", static_cast<int>(position));
    asyn_status = pC_->writeController();

    callParamCallbacks();
    return asyn_status;
}

StatusBits get_status(int status) {
    StatusBits s{};
    s.AmplifiersEnabled = status & (1 << 0);
    s.EndStop = status & (1 << 1);
    s.ThermalProtection1 = status & (1 << 2);
    s.ThermalProtection2 = status & (1 << 3);
    s.ForceZero = status & (1 << 4);
    s.MotorOn = status & (1 << 5);
    s.ClosedLoop = status & (1 << 6);
    s.EncoderAtIndex = status & (1 << 7);
    s.EncoderValid = status & (1 << 8);
    s.SearchingIndex = status & (1 << 9);
    s.PositionReached = status & (1 << 10);
    s.ErrorCompensation = status & (1 << 11);
    return s;
}

std::optional<double> parse_reply(const std::string &str) {
    std::string value_str;
    if (auto ind = str.find('='); ind != std::string::npos) {
        value_str = str.substr(ind + 1);
    }
    return std::stod(value_str);
}

// // Convert from [-180, 180] → [0, 360)
// inline double to360(double angle) {
    // // Shift negatives up by 360
    // if (angle < 0) {
        // angle += 360.0;
    // }
    // return angle;
// }

// // Convert from [0, 360) → [-180, 180)
// inline double to180(double angle) {
    // if (angle > 180.0) {
        // angle -= 360.0;
    // }
    // return angle;
// }

asynStatus XeryonMotorAxis::poll(bool *moving) {
    asynStatus asyn_status = asynSuccess;

    std::optional<double> epos = 0.0;
    std::optional<double> stat = 0.0;
    StatusBits status_bits;

    // encoder position
    sprintf(pC_->outString_, "EPOS=?");
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        goto skip;
    }

    epos = parse_reply(pC_->inString_);
    if (epos.has_value()) {
	const double rbv = epos.value();
        setDoubleParam(pC_->motorPosition_, rbv);
        setDoubleParam(pC_->motorEncoderPosition_, rbv);
    }

    sprintf(pC_->outString_, "STAT=?");
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        goto skip;
    }

    stat = parse_reply(pC_->inString_);
    if (stat.has_value()) {
        status_bits = get_status(stat.value());
        // std::cout << "----------------------------\n";
        // std::cout << "Amplifiers enabled   : " << std::boolalpha << status_bits.AmplifiersEnabled
        // << "\n"; std::cout << "End stop             : " << std::boolalpha << status_bits.EndStop
        // << "\n"; std::cout << "Thermal protection 1 : " << std::boolalpha <<
        // status_bits.ThermalProtection1  << "\n"; std::cout << "Thermal protection 2 : " <<
        // std::boolalpha << status_bits.ThermalProtection2  << "\n"; std::cout << "Force zero : "
        // << std::boolalpha << status_bits.ForceZero           << "\n"; std::cout << "Motor on : "
        // << std::boolalpha << status_bits.MotorOn             << "\n"; std::cout << "Closed loop
        // : " << std::boolalpha << status_bits.ClosedLoop          << "\n"; std::cout << "Encoder
        // at index     : " << std::boolalpha << status_bits.EncoderAtIndex      << "\n"; std::cout
        // << "Encoder valid        : " << std::boolalpha << status_bits.EncoderValid        <<
        // "\n"; std::cout << "Searching index      : " << std::boolalpha <<
        // status_bits.SearchingIndex      << "\n"; std::cout << "Position reached     : " <<
        // std::boolalpha << status_bits.PositionReached     << "\n"; std::cout << "Error
        // compensation   : " << std::boolalpha << status_bits.ErrorCompensation   << "\n";
        // std::cout << "----------------------------\n";
        setIntegerParam(pC_->motorStatusDone_, !status_bits.MotorOn);
        setIntegerParam(pC_->motorStatusMoving_, status_bits.MotorOn);
        *moving = status_bits.MotorOn;

        setIntegerParam(pC_->motorStatusPowerOn_, status_bits.AmplifiersEnabled);
    }

skip:
    callParamCallbacks();
    return asyn_status;
}

asynStatus XeryonMotorAxis::home(double minVelocity, double maxVelocity, double acceleration,
                                 int forwards) {
    asynStatus asyn_status = asynSuccess;

    // set the homing speed which is given as an integer with units 0.01deg/sec
    const int velo = maxVelocity * DRIVER_RESOLUTION * 100; // 0.01 deg/sec
    sprintf(pC_->outString_, "ISPD=%d", velo);
    asyn_status = pC_->writeController();

    // find the index
    sprintf(pC_->outString_, "INDX=%d", static_cast<bool>(forwards) ? 1 : 0);
    asyn_status = pC_->writeController();

    callParamCallbacks();
    return asyn_status;
}

asynStatus XeryonMotorAxis::setClosedLoop(bool closedLoop) {
    asynStatus asyn_status = asynSuccess;

    if (closedLoop) {
        // enables both amplifiers
        sprintf(pC_->outString_, "ENBL=3");
    } else {
        // disables both amplifiers
        sprintf(pC_->outString_, "ENBL=0");
    }
    asyn_status = pC_->writeController();

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
