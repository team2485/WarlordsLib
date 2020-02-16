package frc.team2485.WarlordsLib;

public interface Tunable {

    /**
     * Should run periodically and run the motor to tune when enabled
     * @param enable use this parameter to only enable the motor when this is true.
     */
    void tunePeriodic(boolean enable);

    /**
     * Set the raw PWM of the subsystem motor. This is for setting the PWM WITHOUT using any current-based pwm.
     * @param pwm value between -1 and 1
     */
    void setRawPWM(double pwm);
}
