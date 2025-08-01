#include "asynDriver.h"
#include "asynMotorAxis.h"
#include "asynMotorController.h"
#include <sstream>

class epicsShareClass XeryonMotorAxis : public asynMotorAxis {
  public:
    XeryonMotorAxis(class XeryonMotorController *pC, int axisNo);
    void report(FILE *fp, int level);
    asynStatus stop(double acceleration);
    asynStatus poll(bool *moving);
    asynStatus setClosedLoop(bool closedLoop);
    asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
    asynStatus move(double position, int relative, double minVelocity, double maxVelocity,
                    double acceleration);

  private:
    XeryonMotorController *pC_;
    int axisIndex_;
    friend class XeryonMotorController;
};

class epicsShareClass XeryonMotorController : public asynMotorController {
  public:
    /// \brief Create a new XeryonMotorController object
    ///
    /// \param[in] portName             The name of the asyn port that will be created for this
    /// driver
    /// \param[in] XeryonPortName        The name of the drvAsynIPPort that was created previously
    /// \param[in] numAxes              The number of axes that this controller supports
    /// \param[in] movingPollPeriod     The time between polls when any axis is moving
    /// \param[in] idlePollPeriod       The time between polls when no axis is moving
    XeryonMotorController(const char *portName, const char *XeryonMotorController, int numAxes,
                         double movingPollPeriod, double idlePollPeriod);
    void report(FILE *fp, int level);

    /// \brief Returns a pointer to a XeryonMotorAxis object
    /// \param[in] asynUser structure that encodes the axis index number
    /// \returns NULL if the axis number encoded in pasynUser is invalid
    XeryonMotorAxis *getAxis(asynUser *pasynUser);

    /// \brief Returns a pointer to a XeryonMotorAxis object
    /// \param[in] axisNo Axis index number
    /// \returns NULL if the axis number is invalid
    XeryonMotorAxis *getAxis(int axisNo);

    friend class XeryonMotorAxis;
};
