package frc.team2485.WarlordsLib;

public interface Tunable {

    /**
     * Should run periodically and run the motor to tune
     */
    void tunePeriodic(int layer);

    void setPWM(double pwm);
}
