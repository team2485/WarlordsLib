package frc.team2485.WarlordsLib.motorcontrol;

import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class CurrentSparkMax extends PIDSparkMax implements Manageable {
    private double m_absoluteMaxCurrent;
    private double m_adjustedMaxCurrent;
    public CurrentSparkMax(int deviceID, int maxCurrent) {
        super(deviceID, ControlType.kCurrent);
        this.m_absoluteMaxCurrent = maxCurrent;
        this.m_adjustedMaxCurrent = m_absoluteMaxCurrent;
    }

    public void setPWM(double pwm) {
        this.setCurrent(pwm * this.m_absoluteMaxCurrent);
    }

    public void setCurrent(double current) {
        if(current > m_adjustedMaxCurrent) {
            this.runPID(m_adjustedMaxCurrent);
        } else {
            this.runPID(current);
        }
    }

    public void setAdjustedMaxCurrent(double maxCurrent) {
        this.m_adjustedMaxCurrent = maxCurrent;
    }


    public double getAdjustedMaxCurrent() {
        return this.m_adjustedMaxCurrent;
    }

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("adjusted max current", this::getAdjustedMaxCurrent, null);
    }

}
