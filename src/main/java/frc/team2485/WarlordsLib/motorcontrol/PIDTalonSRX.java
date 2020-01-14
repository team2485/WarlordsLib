package frc.team2485.WarlordsLib.motorcontrol;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ControlType;

public class PIDTalonSRX extends WL_TalonSRX {

    private ControlMode controlMode;

    private double _setpoint;

    private double kP, kI, kD, kIz, kF, kMaxOutput, kMinOutput, kIAccum;

    /**
     * Constructor for TalonSRX object
     *
     * @param deviceNumber CAN Device ID of Device
     */
    public PIDTalonSRX(int deviceNumber, ControlMode controlMode ) {
        super(deviceNumber);

        this.controlMode = controlMode;

        this.kP = this.get


    }
}
