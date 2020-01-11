package frc.team2485.WarlordsLib.motorcontrol;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.team2485.WarlordsLib.robotConfigs.Configurable;
import frc.team2485.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.SavableConfigs;

public class SparkMaxPIDController implements Sendable, Configurable {

    private CANSparkMax _motor;

    private CANPIDController _controller;

    private CANEncoder _encoder;

    private ControlType _controlType;

    private double _setpoint;

    private double kP, kI, kD, kIz, kF, kMaxOutput, kMinOutput, kIAccum;

    public SparkMaxPIDController(CANSparkMax motor, CANEncoder feedbackDevice, ControlType controlType) {

        this._encoder = feedbackDevice;
        this._controlType = controlType;

        this._controller = motor.getPIDController();

        this._controller.setFeedbackDevice(_encoder);
        this._controller.setIAccum(0);

        this.kP = this._controller.getP();
        this.kI = this._controller.getI();
        this.kD = this._controller.getD();
        this.kIz = this._controller.getIZone();
        this.kF = this._controller.getFF();
        this.kMaxOutput = this._controller.getOutputMax();
        this.kMinOutput = this._controller.getOutputMin();
        this.kIAccum = this._controller.getIAccum();
    }

    public double getP() {
        return _controller.getP();
    }

    public void setP(double kP) {
        if (this.kP != kP) {
            _controller.setP(kP);
            this.kP = kP;
        }
    }

    public double getI() {
        return _controller.getI();

    }

    public void setI(double kI) {
        if (this.kI != kI) {
            _controller.setI(kI);
            this.kI = kI;
        }
    }

    public double getD() {
        return _controller.getD();
    }

    public void setD(double kD) {
        if (this.kD != kD) {
            _controller.setD(kD);
            this.kD = kD;
        }
    }

    public double getIzone() {
        return _controller.getIZone();
    }

    public void setIzone(double kIz) {
        if (this.kIz != kIz) {
            _controller.setIZone(kIz);
            this.kIz = kIz;
        }
    }

    public double getFeedforward() {
        return _controller.getFF();
    }

    public void setFeedforward(double kF) {
        if (this.kF != kF) {
            _controller.setFF(kF);
            this.kF = kF;
        }
    }

    public double getMaxOutput() {
        return _controller.getOutputMax();
    }

    public double getMinOutput() {
        return _controller.getOutputMin();
    }

    public double getIAccumulator() {
        return _controller.getIAccum();
    }

    public void setIAccumulator(double kIAccum) {
        if (this.kIAccum != kIAccum) {
            _controller.setIAccum(kIAccum);
            this.kIAccum = kIAccum;
        }
    }

    public void setPID(double p, double i,double d) {
        this.setP(p);
        this.setI(i);
        this.setD(d);
    }

    public void setPIDF(double p, double i, double d, double f) {
        if ((this.kP != kP) || (this.kI != kI) || (this.kD != kD) || (this.kF != kF)) {
            _controller.setP(p);
            _controller.setI(i);
            _controller.setD(d);
        }
    }

    public void setOutputRange(double kMinOutput, double kMaxOutput) {
        if ((this.kMinOutput != kMinOutput) || (this.kMaxOutput != kMaxOutput)) {
            _controller.setOutputRange(kMinOutput, kMaxOutput);
            this.kMinOutput = kMinOutput;
            this.kMaxOutput = kMaxOutput;
        }
    }

    /**
     * Get output depending on the {@link ControlType}
     * @return output of pid
     */
    public double getOutput()  {
        switch (_controlType) {
            case kCurrent:
                return _motor.getOutputCurrent();
            case kSmartVelocity:
            case kVelocity:
                return _encoder.getVelocity();
            case kSmartMotion:
            case kPosition:
                return _encoder.getPosition();
            case kVoltage:
                return _motor.getBusVoltage();
            case kDutyCycle:
                return _motor.getAppliedOutput();
            default:
                return 0;
        }
    }

    /**
     *
     * @return the control type the PIDController is using (current, velocity, position)
     */
    public ControlType getControlType() {
        return this._controlType;
    }

    public void setControlType(ControlType controlType) {
        this._controlType = controlType;
    }

    /**
     * Set the target for Talon PIDController.
     * @param target
     */
    public void setReference(double target) {
        setSetpoint(target);
        _controller.setReference(_setpoint, _controlType);
    }

    public void setSetpoint(double setpoint) {
        this._setpoint = setpoint;
    }

    public double getSetpoint() {
        return this._setpoint;
    }

    public void reset() {
        _controller.setIAccum(0);
    }

    public CANPIDController getPIDController() {
        return this._controller;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("f", this::getFeedforward, this::setFeedforward);
        builder.addDoubleProperty("iZone", this::getIzone, this::setIzone);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);

        builder.addDoubleProperty("output", this::getOutput, null);
    }

    @Override
    public void loadConfigs(LoadableConfigs configs) {
    }

    @Override
    public void saveConfigs(SavableConfigs configs) {

    }

}
