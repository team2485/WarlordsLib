package frc.team2485.WarlordsLib.motorcontrol;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.team2485.WarlordsLib.robotConfigs.Configurable;
import frc.team2485.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.SavableConfigs;

import static com.revrobotics.ControlType.*;

public class PIDSparkMax extends WL_SparkMax implements Configurable {

    private ControlType _controlType;

    private CANPIDController _controller;

    private CANEncoder _encoder;

    private double _setpoint;

    private double kP, kI, kD, kIz, kF, kMaxOutput, kMinOutput, kIMaxAccum;
    //Smart motion variables
    private double minVel, maxVel, maxAcc, allowedError;
    private CANPIDController.AccelStrategy accelStrategy;

    /**
     * Create a new Brushless SPARK MAX Controller
     *
     * @param deviceID The device ID.
     */
    public PIDSparkMax(int deviceID, ControlType controlType) {
        super(deviceID);

        this._controlType = controlType;

        this._encoder = this.getEncoder();

        this._controller = this.getPIDController();

        this._controller.setIAccum(0);

        this.kP = this._controller.getP();
        this.kI = this._controller.getI();
        this.kD = this._controller.getD();
        this.kIz = this._controller.getIZone();
        this.kF = this._controller.getFF();
        this.kMaxOutput = this._controller.getOutputMax();
        this.kMinOutput = this._controller.getOutputMin();
        this.kIMaxAccum = this._controller.getIMaxAccum(0);
        this.maxVel = this._controller.getSmartMotionMaxVelocity(0);
        this.minVel = this._controller.getSmartMotionMinOutputVelocity(0);
        this.maxAcc = this._controller.getSmartMotionMaxAccel(0);
        this.allowedError = this._controller.getSmartMotionAllowedClosedLoopError(0);
        this.accelStrategy = this._controller.getSmartMotionAccelStrategy(0);

    }

    public void setFeedbackDevice(CANEncoder feedbackDevice) {
        this._encoder = feedbackDevice;
        _controller.setFeedbackDevice(feedbackDevice);
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

    public double getIMaxAccum() {
        return _controller.getIMaxAccum(0);
    }

    public void setIMaxAccum(double kIMaxAccum) {
        if (this.kIMaxAccum != kIMaxAccum) {
            _controller.setIMaxAccum(kIMaxAccum, 0);
            this.kIMaxAccum = kIMaxAccum;
        }
    }

    public double getF() {
        return _controller.getFF();
    }

    public void setF(double kF) {
        if (this.kF != kF) {
            _controller.setFF(kF);
            this.kF = kF;
        }
    }

    public double getMaxVel() {
        return this.maxVel;
    }

    public void setMaxVel(double maxVel) {
        if (maxVel != this.maxVel) {
            _controller.setSmartMotionMaxVelocity(maxVel, 0);
            this.maxVel = maxVel;
        }

    }

    public double getMinVel() {
        return this.minVel;
    }

    public void setMinVel(double minVel) {
        if (minVel != this.minVel){
            _controller.setSmartMotionMinOutputVelocity(minVel, 0);
            this.minVel = minVel;
        }
    }

    public double getMaxAcc() {
        return this.maxAcc;
    }

    public void setMaxAcc(double maxAcc) {
        if (maxAcc != this.maxAcc) {
            _controller.setSmartMotionMaxAccel(maxAcc, 0);
            this.maxAcc = maxAcc;
        }

    }

    public double getAllowedError() {
        return this.allowedError;
    }
    public void setAllowedError(double allowedError) {
        if (allowedError != this.allowedError) {
            _controller.setSmartMotionAllowedClosedLoopError(allowedError, 0);
            this.allowedError = allowedError;
        }

    }

    public void setAccelStrategy(CANPIDController.AccelStrategy accelStrategy) {
        _controller.setSmartMotionAccelStrategy(accelStrategy, 0);
        this.accelStrategy = accelStrategy;
    }

    public CANPIDController.AccelStrategy getAccelStrategy() {
        return this.accelStrategy;
    }

    public double getMaxOutput() {
        return _controller.getOutputMax();
    }

    public double getMinOutput() {
        return _controller.getOutputMin();
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

    public double getOutput(ControlType controlType) {
        switch (controlType) {
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
        builder.addDoubleProperty("f", this::getF, this::setF);
        builder.addDoubleProperty("iZone", this::getIzone, this::setIzone);
        builder.addDoubleProperty("iMaxAccum", this::getIMaxAccum, this::setIMaxAccum);
        builder.addDoubleProperty("rampRate", this::getClosedLoopRampRate, this::setClosedLoopRampRate);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
        builder.addDoubleProperty("output", this::getOutput, null);
        builder.addDoubleProperty("PWM output", this::get, null);
        if (this._controlType != kCurrent) {
            builder.addDoubleProperty("Current output", this::getOutputCurrent, null);
        }

        if(this._controlType == kSmartMotion){
            builder.addDoubleProperty("Max velocity", this::getMaxVel, this::setMaxVel);
            builder.addDoubleProperty("Min velocity", this::getMinVel, this::setMinVel);
            builder.addDoubleProperty("Max acceleration", this::getMaxAcc, this::setMaxAcc);
            builder.addDoubleProperty("Allowed error", this::getAllowedError,this::setAllowedError);
        }


    }

    @Override
    public void loadConfigs(LoadableConfigs configs) {
        this.setP(configs.getDouble("p", this.getP()));
        this.setI(configs.getDouble("i", this.getI()));
        this.setD(configs.getDouble("d", this.getD()));
        this.setF(configs.getDouble("f", this.getF()));
        this.setIzone(configs.getDouble("iZone", this.getIzone()));
        this.setIMaxAccum(configs.getDouble("iMaxAccum", this.getIMaxAccum()));
    }

    @Override
    public void saveConfigs(SavableConfigs configs) {
        configs.put("p", this.getP());
        configs.put("i", this.getI());
        configs.put("d", this.getD());
        configs.put("f", this.getF());
        configs.put("iZone", this.getIzone());
        configs.put("iMaxAccum", this.getIMaxAccum());
    }


}
