package frc.team2485.WarlordsLib;

public interface Tunable {

    /**
     * Should run periodically and run the motor to tune when enabled
     * @param enable use this parameter to only enable the motor when this is true.
     */
    void tunePeriodic(boolean enable);
}
