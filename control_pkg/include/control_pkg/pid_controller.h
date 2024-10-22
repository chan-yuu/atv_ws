#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

namespace control
{

struct PidConf {
    int integrator_enable;
    double integrator_saturation_level;
    double output_saturation_level;
    double kp;
    double ki;
    double kd;
    double kaw;
    double nts;
    PidConf() :
        integrator_enable(0),
        integrator_saturation_level(0),
        output_saturation_level(0),
        kp(0),
        ki(0),
        kd(0),
        kaw(0),
        nts(0)
    {}
};

class PIDController
{
public:
    /**
     * @brief initialize pid controller
     * @param pid_conf configuration for pid controller
     */
    void Init(const PidConf &pid_conf);

    /**
     * @brief set pid controller coefficients for the proportional,
     * integral, and derivative
     * @param pid_conf configuration for pid controller
     */
    // void SetPID(const PidConf &pid_conf);

    /**
     * @brief reset variables for pid controller
     */
    void Reset();

    /**
     * @brief compute control value based on the error
     * @param error error value, the difference between
     * a desired value and a measured value
     * @return control value based on PID terms
     */
    virtual double Control(const double error);

    virtual ~PIDController() = default;

    /**
     * @brief get saturation status
     * @return saturation status
     */
    int IntegratorSaturationStatus() const;

    /**
     * @brief get status that if integrator is hold
     * @return if integrator is hold return true
     */
    bool IntegratorHold() const;

    /**
     * @brief set whether to hold integrator component at its current value.
     * @param hold
     */
    void SetIntegratorHold(bool hold);

protected:
    double kp_ = 0.0;
    double ki_ = 0.0;
    double kd_ = 0.0;
    double nts_ = 0.0;
    double kaw_ = 0.0;
    double previous_error_ = 0.0;
    double previous_output_ = 0.0;
    double integral_ = 0.0;
    double integrator_saturation_high_ = 0.0;
    double integrator_saturation_low_ = 0.0;
    bool first_hit_ = false;
    bool integrator_enabled_ = false;
    bool integrator_hold_ = false;
    int integrator_saturation_status_ = 0;
    // Only used for pid_BC_controller and pid_IC_controller
    double output_saturation_high_ = 0.0;
    double output_saturation_low_ = 0.0;
    int output_saturation_status_ = 0;
};

}  // namespace control

#endif
