package frc.team2485.WarlordsLib.motorcontrol;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.team2485.WarlordsLib.robotConfigs.Configurable;
import frc.team2485.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.SavableConfigs;

public class PIDSparkMax extends WL_SparkMax implements Configurable {

    private ControlType _controlType;

    private CANPIDController _controller;

    private CANEncoder _encoder;

    private double _setpoint;

    private double kP, kI, kD, kIz, kF, kMaxOutput, kMinOutput, kIAccum;

    /**
     * Create a new Brushless SPARK MAX Controller
     *
     * @param deviceID The device ID.
     */
    public PIDSparkMax(int deviceID, ControlType controlType, CANEncoder feedbackReference) {
        super(deviceID);

        this._controlType = controlType;

        this._encoder = feedbackReference;

        this._controller.setFeedbackDevice(_encoder);

        this._controller = this.getPIDController();

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
     *
     * @return the control type the PIDController is using (current, velocity, position)
     */
    public ControlType getControlType() {
        return this._controlType;
    }

    public void setControlType(ControlType controlType) {
        this._controlType = controlType;
    }

    public void setReference() {
        _controller.setReference(_setpoint, _controlType);
    }

    public void setReference(double target) {
        setSetpoint(target);
        setReference();
    }

    public CANPIDController getController() {
        return this._controller;
    }

    /**
     * Set the controller reference value based on the selected control mode.
     *
     * @param setpoint The value to set depending on the control mode. For basic
     * duty cycle control this should be a value between -1 and 1
     * Otherwise: Voltage Control: Voltage (volts) Velocity Control: Velocity
     * (RPM) Position Control: Position (Rotations) Current Control: Current
     * (Amps). Native units can be changed using the setPositionConversionFactor()
     * or setVelocityConversionFactor() methods of the CANEncoder class
     *
     *
     */
    public void setSetpoint(double setpoint) {
        this._setpoint = setpoint;
    }

    public double getSetpoint() {
        return this._setpoint;
    }

    public void reset() {
        this._controller.setIAccum(0);
    }

    public void zero() {
        _encoder.setPosition(0);
    }

    /**
     * Get output depending on the {@link ControlType}
     * @return output of pid
     */
    public double getOutput()  {
        switch (_controlType) {
            case kCurrent:
                return this.getOutputCurrent();
            case kSmartVelocity:
            case kVelocity:
                return _encoder.getVelocity();
            case kSmartMotion:
            case kPosition:
                return _encoder.getPosition();
            case kVoltage:
                return this.getBusVoltage();
            case kDutyCycle:
                return this.getAppliedOutput();
            default:
                return 0;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
//        builder.setSmartDashboardType("PIDController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("f", this::getFeedforward, this::setFeedforward);
        builder.addDoubleProperty("iZone", this::getIzone, this::setIzone);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }

    @Override
    public void loadConfigs(LoadableConfigs configs) {
    }

    @Override
    public void saveConfigs(SavableConfigs configs) {

    }


}
