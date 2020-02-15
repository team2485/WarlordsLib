package frc.team2485.WarlordsLib.motorcontrol;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class CurrentTalonSRX extends PIDTalonSRX implements Manageable {
    private double m_absoluteMaxCurrent;
    private double adjustedMaxCurrent;
    public CurrentTalonSRX(int deviceID, int maxCurrent) {
        super(deviceID, ControlMode.Current);
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

    public void setAdjustedMaxCurrent(double maxCurrent) {
        this.adjustedMaxCurrent = maxCurrent;
    }


    public double getAdjustedMaxCurrent() {
        return this.adjustedMaxCurrent;
    }

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("adjusted max current", this::getAdjustedMaxCurrent, null);
    }

}
