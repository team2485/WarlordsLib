package frc.team2485.WarlordsLib.motorcontrol;

import com.revrobotics.ControlType;

public class CurrentSparkMax extends PIDSparkMax implements Manageable {
    private double m_absoluteMaxCurrent;
    private double adjustedMaxCurrent;
    public CurrentSparkMax(int deviceID, int maxCurrent) {
        super(deviceID, ControlType.kCurrent);
        this.m_absoluteMaxCurrent = maxCurrent;
        this.adjustedMaxCurrent = m_absoluteMaxCurrent;
    }

    public void setPWM(double pwm) {
        this.setCurrent(pwm * this.m_absoluteMaxCurrent);
    }

    public void setCurrent(double current) {
        if(current > adjustedMaxCurrent) {
            this.runPID(adjustedMaxCurrent);
        } else {
            this.runPID(current);
        }
    }

    @Override
    public void setAdjustedMaxCurrent(double maxCurrent) {
        this.adjustedMaxCurrent = maxCurrent;
    }
}
