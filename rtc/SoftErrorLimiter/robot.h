#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
#include <boost/intrusive_ptr.hpp>

// robot model copy from RobotHardware
class robot : public hrp::Body
{
public:
    /**
       \brief constructor
     */
    robot();

    /**
       \brief destructor
    */
    ~robot();

    /**
       \brief
     */
    bool init();

    /**
       \brief set servo error limit value for specific joint or joint group
       \param i_jname joint name or joint group name
       \param i_limit new limit value[rad]
       \return true if set successfully, false otherwise 
     */
    bool setServoErrorLimit(const char *i_jname, double i_limit);

    /**
       \brief set torque limit value for specific joint or joint group
       \param i_jname joint name or joint group name
       \param i_limit new limit value[Nm]
       \return true if set successfully, false otherwise 
     */
    bool settauLimit(const char *i_jname, double i_limit);

    /**
       \brief set servo velocity limit value for specific joint or joint group
       \param i_jname joint name or joint group name
       \param i_limit new limit value[rad/s]
       \return true if set successfully, false otherwise 
     */
    bool setVelocityLimit(const char *i_jname, double i_limit);

    /**
       \brief set ispassive(no positioncheck) for specific joint or joint group
       \param i_jname joint name or joint group name
       \param i_ispassive new value[bool]
       \return true if set successfully, false otherwise 
     */
    bool setpassivejoint(const char *i_jname, bool i_ispassive);

    //boost::interprocess::interprocess_semaphore wait_sem;

    std::vector<double> m_servoErrorLimit;
    std::vector<double> m_tauLimit;
    std::vector<double> m_VelocityLimit;
    std::vector<bool> m_passivejoints;
};
