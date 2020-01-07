package frc.team2485.WarlordsLib.motorcontrol;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.team2485.WarlordsLib.motorcontrol.base.WPI_SparkMax;
import frc.team2485.WarlordsLib.robotConfigs.ConfigurableBuilder;

import javax.naming.ldap.Control;

/**
 * Warlords wrapper for Spark Max with convenience functions and PIDF control.
 *
 */
public class WL_SparkMax extends WPI_SparkMax {
    private CANPIDController m_pidController;
    private CANEncoder m_encoder;
    private ControlType m_controlType;
    private double m_setpoint;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    /**
     * Create a new SPARK MAX Controller
     *
     * @param deviceID The device ID.
     * @param type     The motor type connected to the controller. Brushless motors
     *                 must be connected to their matching color and the hall sensor
     */
    public WL_SparkMax(int deviceID, MotorType type) {
        super(deviceID, type);
        this.restoreFactoryDefaults();
    }

    /**
     * Create a new SPARK MAX Controller with PID control
     *
     * @param deviceID The device ID.
`     * @param type     The motor type connected to the controller. Brushless motors
`     *                 must be connected to their matching color and the hall sensor
     * @param controlType The type of onboard PIDcontrol (kCurrent, kVelocity, kPosition, etc.)
     */
    public WL_SparkMax(int deviceID, MotorType type, ControlType controlType) {
        super(deviceID, type);
        this.m_controlType = controlType;
        this.m_pidController = this.getPidController();
        this.m_encoder = this.getEncoder();
        m_setpoint = 0;
        //default PID values
        kP = 0.1;
        kI = 1e-4;
        kD = 1;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 1;
        kMinOutput = -1;

        // set PID coefficients
        this.m_pidController.setP(kP);
        this.m_pidController.setI(kI);
        this.m_pidController.setD(kD);
        this.m_pidController.setIZone(kIz);
        this.m_pidController.setFF(kFF);
        this.m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        this.restoreFactoryDefaults();
    }

    /**
     * Create a new Brushless SPARK MAX Controller
     *
     * @param deviceID The device ID.
     */
    public WL_SparkMax(int deviceID) {
        this(deviceID, MotorType.kBrushless);
    }

    /** Run the onboard PID controller.
     *
     * @param setpoint the setpoint (of m_controlType) the PID controller is set to
     */
    public void runPID(double setpoint) {
        this.m_setpoint = setpoint;
        this.m_pidController.setReference(m_setpoint, m_controlType);
    }

    /** Run the onboard pid controller, using the existing setpoint.
     *
     */
    public void runPID() {
        this.m_pidController.setReference(m_setpoint, m_controlType);
    }

    /**
     *
     * @param setpoint the setpoint (of m_controlType) the PID controller is set to
     */
    public void setSetpoint(double setpoint) {
        this.m_setpoint = setpoint;
    }

    /**
     *
     * @return the (m_controlType) setpoint of the PID controller
     */
    public double getSetpoint() {
        return this.m_setpoint;
    }

    /**
     *
     * @return the output of the PID controller, based on the control type
     */
    public double getOutput() {
        switch (m_controlType) {
            case kPosition:
                return m_encoder.getPosition();
            case kVelocity:
                return m_encoder.getVelocity();
            case kCurrent:
                return this.getOutputCurrent();
            default:
                return 0;
        }
    }

    /**
     *
     * @return the control type the PIDController is using (current, velocity, position)
     */
    public ControlType getControlType() {
        return this.m_controlType;
    }

    /**
     * Sets other sparks to follow this Spark.
     * @param slave the follower motor
     * @param slaves any number of follower motors
     */
    public void setFollowers(WL_SparkMax slave, WL_SparkMax... slaves) {
        slave.follow(this);
        for (CANSparkMax m : slaves) {
            m.follow(this);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSafeState(this::disable);
        builder.addDoubleProperty("P Term", m_pidController::getP, m_pidController::setP);
        builder.addDoubleProperty("I Term", m_pidController::getI, m_pidController::setI)
        builder.addDoubleProperty("D Term", m_pidController::getD, m_pidController::setD)
        builder.addDoubleProperty("F Term", m_pidController::getFF, m_pidController::setFF)
        builder.addDoubleProperty("IZone Term", m_pidController::getIZone, m_pidController::setIZone);
        builder.addDoubleProperty("Setpoint", this::getSetpoint, this::setSetpoint);
        builder.addDoubleProperty("Output", this::getOutput, this::setSetpoint);
        }
    }

    public void initConfigurable(ConfigurableBuilder builder) {
        builder.addDoubleProperty("p", m_pidController::getP, m_pidController::setP);
        builder.addDoubleProperty("i", m_pidController::getI, m_pidController::setI)
        builder.addDoubleProperty("d", m_pidController::getD, m_pidController::setD)
        builder.addDoubleProperty("f", m_pidController::getFF, m_pidController::setFF)
        builder.addDoubleProperty("iZone", m_pidController::getIZone, m_pidController::setIZone);
    }


}
