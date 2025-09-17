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
constexpr int NUM_PARAMS = 6;

// Parse reply from commands sent from the controller
// E.g. send "EPOS=?", receive "EPOS=1000".
// parse_reply("EPOS=1000") returns 1000 as an optional<int>
std::optional<int> parse_reply(const std::string &str) {
    if (auto ind = str.find('='); ind != std::string::npos) {
        try {
            return std::stoi(str.substr(ind + 1));
        } catch (...) {
            return std::nullopt; // invalid integer
        }
    }
    return std::nullopt; // '=' not found
}

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

    createParam(FREQUENCY1_STRING, asynParamInt32, &frequency1Index_);
    createParam(FREQUENCY2_STRING, asynParamInt32, &frequency2Index_);
    createParam(READ_PARAMS_STRING, asynParamInt32, &readParamsIndex_);
    createParam(CONTROL_TIMEOUT_STRING, asynParamInt32, &controlTimeoutIndex_);
    createParam(CONTROL_TIMEOUT2_STRING, asynParamInt32, &controlTimeout2Index_);
    createParam(CONTROL_FREQUENCY_STRING, asynParamInt32, &controlFreqIndex_);
    createParam(POS_TOLERANCE_STRING, asynParamInt32, &posToleranceIndex_);
    createParam(POS_TOLERANCE2_STRING, asynParamInt32, &posTolerance2Index_);
    createParam(STATUS_BITS_STRING, asynParamInt32, &statusBitsIndex_);
    createParam(ZONE1_STRING, asynParamInt32, &zone1Index_);
    createParam(ZONE2_STRING, asynParamInt32, &zone2Index_);

    // Map controller command strings to the associated asyn parameter index
    cmd_param_map_ = std::unordered_map<std::string, int> {
	{"FREQ", frequency1Index_},
	{"FRQ2", frequency2Index_},
	{"CFRQ", controlFreqIndex_},
	{"PTOL", posToleranceIndex_},
	{"PTO2", posTolerance2Index_},
	{"TOUT", controlTimeoutIndex_},
	{"TOU2", controlTimeout2Index_},
	{"ZON1", zone1Index_},
	{"ZON2", zone2Index_},
    };

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

asynStatus XeryonMotorController::writeInt32(asynUser *pasynUser, epicsInt32 value) {

    int function = pasynUser->reason;
    asynStatus asyn_status = asynSuccess;
    XeryonMotorAxis *pAxis;

    pAxis = this->getAxis(pasynUser);
    if (!pAxis) {
        return asynError;
    };

    if (function == readParamsIndex_) {
        asyn_status = pAxis->update_params();
        if (asyn_status) {
            goto skip;
        }
    } else {
	for (const auto &[cmd, param_index] : cmd_param_map_) {
	    if (function == param_index) {
		sprintf(outString_, "%s=%d", cmd.c_str(), value);
		asyn_status = this->writeController();
		if (asyn_status) {
		    goto skip;
		}
	    }
	}
    }

skip:
    pAxis->callParamCallbacks();
    callParamCallbacks();
    return asyn_status;
}

asynStatus XeryonMotorAxis::update_params() {
    asynStatus asyn_status = asynSuccess;

    for (auto &[cmd, param_index] : pC_->cmd_param_map_) {
	sprintf(pC_->outString_, "%s=?", cmd.c_str());
	asyn_status = pC_->writeReadController();
	if (asyn_status) {
	    return asyn_status;
	}
	auto ret = parse_reply(pC_->inString_);
	if (ret.has_value()) {
	    pC_->setIntegerParam(param_index, ret.value());
	}
    }
    return asyn_status;
    // assume caller calls callParamCallbacks()
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
    if (asyn_status) {
	goto skip;
    }

    // Move to this target position in closed loop
    sprintf(pC_->outString_, "DPOS=%d", static_cast<int>(position));
    asyn_status = pC_->writeController();
    if (asyn_status) {
	goto skip;
    }

skip:
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

asynStatus XeryonMotorAxis::poll(bool *moving) {
    asynStatus asyn_status = asynSuccess;

    std::optional<int> epos = 0;
    std::optional<int> stat = 0;
    StatusBits status_bits;

    // Encoder position
    sprintf(pC_->outString_, "EPOS=?");
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        goto skip;
    }
    epos = parse_reply(pC_->inString_);
    if (epos.has_value()) {
        int rbv = epos.value();
        if (rbv > 28800) {
	    // keep readback in range -180, 180
            rbv = rbv - 57600;
        }
        setDoubleParam(pC_->motorPosition_, rbv);
        setDoubleParam(pC_->motorEncoderPosition_, rbv);
    }

    // get status word
    sprintf(pC_->outString_, "STAT=?");
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        goto skip;
    }
    stat = parse_reply(pC_->inString_);
    if (stat.has_value()) {
	setIntegerParam(pC_->statusBitsIndex_, stat.value());
        status_bits = get_status(stat.value());
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
    if (asyn_status) {
	goto skip;
    }

    // find the index
    sprintf(pC_->outString_, "INDX=%d", static_cast<bool>(forwards) ? 1 : 0);
    asyn_status = pC_->writeController();
    if (asyn_status) {
	goto skip;
    }

skip:
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
