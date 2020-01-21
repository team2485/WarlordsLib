package frc.team2485.WarlordsLib.motorcontrol.base;

public interface PIDMotorController {

    /**
     * Set proportional coefficient
     * @param p proportional coefficient
     */
    void setP(double p);

    /**
     * Get proportional coefficient
     * @return proportional coefficient
     */
    double getP();

    /**
     * Set integrator coefficient
     * @param i integral coefficient
     */
    void setI(double i);

    /**
     * Get integrator coefficient
     * @return integrator coefficient
     */
    double getI();

    /**
     * Set derivative coefficient
     * @param d derivative coefficient
     */
    void setD(double d);

    /**
     * Get derivative coeffficient
     * @return derivative coefficient
     */
    double getD();

    /**
     * Set setpoint (does not set actual motor)
     * @param setpoint setpoint of pid
     */
    void setSetpoint(double setpoint);

    /**
     * Get setpoint
     * @return setpoint
     */
    double getSetpoint();

    /**
     * Run the PID of the motor controller based on the setpoint
     */
    void runPID();

    /**
     * Get output of PID based on mode
     * @return pid output
     */
    double getOutput();

    /**
     * Reset integrators and pid here.
     */
    void reset();

    /**
     * Reset encoder position here
     * @param position set encoder position
     */
    void setEncoderPosition(double position);
}
