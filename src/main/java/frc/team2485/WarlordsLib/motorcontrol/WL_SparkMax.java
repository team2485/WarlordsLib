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
    private CANPIDController pidController;
    private CANEncoder encoder;
    private ControlType controlType;
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
     * @param type     The motor type connected to the controller. Brushless motors
     *                 must be connected to their matching color and the hall sensor
     * @param controlType The type of onboard PIDcontrol (kCurrent, kVelocity, kPosition, etc.)
     */
    public WL_SparkMax(int deviceID, MotorType type, ControlType controlType) {
        super(deviceID, type);
        this.controlType = controlType;
        this.pidController = this.getPIDController();
        this.encoder = this.getEncoder();
        //default PID values
        kP = 0.1;
        kI = 1e-4;
        kD = 1;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 1;
        kMinOutput = -1;

        // set PID coefficients
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

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

    public void runPID() {

    }

    public void setFollowers(WL_SparkMax slave, WL_SparkMax... slaves) {
        slave.follow(this);
        for (CANSparkMax m : slaves) {
            m.follow(this);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSafeState(this::disable);
        builder.addDoubleProperty("P Term", pidController::getP, pidController::setP);
        builder.addDoubleProperty("I Term", pidController::getI, pidController::setI)
        builder.addDoubleProperty("D Term", pidController::getD, pidController::setD)
        builder.addDoubleProperty("F Term", pidController::getFF, pidController::setFF)
        builder.addDoubleProperty("IZone Term", pidController::getIZone, pidController::setIZone);
    }

    public void initConfigurable(ConfigurableBuilder builder) {
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("f", this::getF, this::setF);
    }


}
