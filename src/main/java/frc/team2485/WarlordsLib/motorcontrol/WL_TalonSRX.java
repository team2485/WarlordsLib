package frc.team2485.WarlordsLib.motorcontrol;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.team2485.WarlordsLib.sensors.TalonSRXEncoder;

/**
 * Warlords wrapper for TalonSRX with convenience functions.
 */
public class WL_TalonSRX extends WPI_TalonSRX {

    /**
     * Constructor for TalonSRX object
     *
     * @param deviceNumber CAN Device ID of Device
     */
    public WL_TalonSRX(int deviceNumber) {
        super(deviceNumber);
        this.configFactoryDefault();
        this.clearStickyFaults();
        this.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * The given motor controllers will now follow this motor controller.
     */
    public void setFollowers(BaseMotorController slave, BaseMotorController... slaves) {
        slave.follow(this);
        for (BaseMotorController m : slaves) {
            m.follow(this);
        }
    }

    /**
     * Config all current limits
     * @param peakCurrentLimit Peak current limit
     * @param peakCurrentDuration Peak current limit duration (milliseconds)
     * @param continuousCurrentLimit Continuous current limit
     * @param currentLimitsEnabled Enable state of current limit.
     */
    public void setCurrentLimiting(int peakCurrentLimit, int peakCurrentDuration, int continuousCurrentLimit, boolean currentLimitsEnabled) {
        this.configPeakCurrentLimit(peakCurrentLimit);
        this.configPeakCurrentDuration(peakCurrentDuration);
        this.configContinuousCurrentLimit(continuousCurrentLimit);
        this.enableCurrentLimit(currentLimitsEnabled);
    }

    /**
     * Config all current limits
     * @param peakCurrentLimit Peak current limit
     * @param peakCurrentDuration Peak current limit duration (milliseconds)
     * @param continuousCurrentLimit Continuous current limit
    s */
    public void setCurrentLimiting(int peakCurrentLimit, int peakCurrentDuration, int continuousCurrentLimit) {
        setCurrentLimiting(peakCurrentLimit, peakCurrentDuration, continuousCurrentLimit, true);
    }

    public void enableVoltageCompensation(double voltage) {
        this.configVoltageCompSaturation(voltage);
        this.enableVoltageCompensation(true);
    }

    public void enableVoltageCompensation() {
        this.enableVoltageCompensation(12.0);
    }

    public TalonSRXEncoder getEncoder(TalonSRXEncoder.TalonSRXEncoderType encoderType, int pulsesPerRevolution) {
        return new TalonSRXEncoder(this, encoderType, pulsesPerRevolution);
    }
}
