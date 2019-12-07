package frc.team2485.WarlordsLib.motorcontrol;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class WL_TalonSRX extends WPI_TalonSRX {

    /**
     * Constructor for TalonSRX object
     *
     * @param deviceNumber CAN Device ID of Device
     */
    public WL_TalonSRX(int deviceNumber) {
        super(deviceNumber);
    }

    public WL_TalonSRX(int deviceNumber, boolean isInverted) {
        super(deviceNumber);
        this.setInverted(isInverted);
    }


    public void setFollowers(BaseMotorController slave, BaseMotorController... slaves) {
        slave.follow(this);
        for (BaseMotorController m : slaves) {
            m.follow(this);
        }
    }

    public void setCurrentLimiting(int peakCurrentLimit, int peakCurrentDuration, int continuousCurrentLimit, boolean currentLimitsEnabled) {
        this.configPeakCurrentLimit(peakCurrentLimit);
        this.configPeakCurrentDuration(peakCurrentDuration);
        this.configContinuousCurrentLimit(continuousCurrentLimit);
        this.enableCurrentLimit(currentLimitsEnabled);
    }




}
