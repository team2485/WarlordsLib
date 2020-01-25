package frc.team2485.WarlordsLib.motorcontrol;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import com.revrobotics.ControlType;
import frc.team2485.WarlordsLib.robotConfigs.Configurable;
import frc.team2485.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.SavableConfigs;

public class PIDTalonSRX extends WL_TalonSRX {

    private ControlMode controlMode;

    private double _setpoint;

    private double kP, kI, kD, kIz, kF, kMaxOutput, kMinOutput, kIMaxAccum;

    private int pidIdx = 0;

    /**
     * Constructor for TalonSRX object
     *
     * @param deviceNumber CAN Device ID of Device
     */
    public PIDTalonSRX(int deviceNumber, ControlMode controlMode) {
        super(deviceNumber);

        this.controlMode = controlMode;




    }

    public void setP(double kP) {
        this.config_kP(pidIdx, kP);
    }

    public void setI(double kI) {
        this.config_kP(pidIdx, kI);
    }

    public void setD(double kD) {
        this.config_kD(pidIdx, kD);
    }

    public void setF(double kF) {
        this.config_kF(pidIdx, kF);
    }

    public void setIzone(double kIz) {
        this.config_IntegralZone(pidIdx, (int) kIz);
    }

    public void setIMaxAccum(double kIMaxAccum) {
        this.configMaxIntegralAccumulator(pidIdx, kIMaxAccum);
    }

    public void setSetpoint(double setpoint) {
        this._setpoint = setpoint;
    }

    public double getSetpoint() {
        return this._setpoint;
    }

    public void setReference() {
        this.set(controlMode, _setpoint);
    }

    public void setReference(double target) {
        this.setSetpoint(target);
        this.setReference();
    }

    public ControlMode getControlMode() {
        return this.controlMode;
    }

    public void setControlMode(ControlMode controlMode) {
        this.controlMode = controlMode;
    }

    public void setOutputRange(double minOutput, double MaxOutput) {

    }

    public double getOutput() {
        switch (controlMode) {
            case Current:
                return this.getSupplyCurrent();
            case Velocity:
                return this.getSelectedSensorVelocity();
            case Position:
                return this.getSelectedSensorPosition();
            case PercentOutput:
                return this.getMotorOutputPercent();
            default:
                return 0;
        }
    }


}
