package frc.team2485.WarlordsLib.motorcontrol;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.team2485.WarlordsLib.motorcontrol.base.PIDMotorController;
import frc.team2485.WarlordsLib.robotConfigs.Configurable;
import frc.team2485.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.SavableConfigs;

public class PIDSparkMax extends WL_SparkMax implements Configurable, PIDMotorController {

    private ControlType m_controlType;

    private CANPIDController m_controller;

    private CANEncoder m_encoder;

    private double m_setpoint;

    private double m_kP, m_kI, m_kD, m_kIz, m_kF, m_kMaxOutput, m_kMinOutput, m_kIMaxAccum;

    /**
     * Create a new Brushless SPARK MAX Controller
     *
     * @param deviceID The device ID.
     */
    public PIDSparkMax(int deviceID, ControlType controlType) {
        super(deviceID);

        this.m_controlType = controlType;

        this.m_encoder = this.getEncoder();

        this.m_controller = this.getPIDController();

        this.m_controller.setIAccum(0);

        this.m_kP = this.m_controller.getP();
        this.m_kI = this.m_controller.getI();
        this.m_kD = this.m_controller.getD();
        this.m_kIz = this.m_controller.getIZone();
        this.m_kF = this.m_controller.getFF();
        this.m_kMaxOutput = this.m_controller.getOutputMax();
        this.m_kMinOutput = this.m_controller.getOutputMin();
        this.m_kIMaxAccum = this.m_controller.getIMaxAccum(0);
    }

    public void setFeedbackDevice(CANEncoder feedbackDevice) {
        this.m_encoder = feedbackDevice;
        m_controller.setFeedbackDevice(feedbackDevice);
    }

    /**
     * gets the proportional coefficient
     *
     * @return proportional coefficient
     */
    @Override
    public double getP() {
        return m_controller.getP();
    }

    /**
     * sets proportional coefficient
     *
     * @param kP proportional coefficient
     */
    @Override
    public void setP(double kP) {
        if (this.m_kP != kP) {
            m_controller.setP(kP);
            this.m_kP = kP;
        }
    }

    /**
     * gets the Integral coefficient
     *
     * @return integral coefficient
     */
    @Override
    public double getI() {
        return m_controller.getI();

    }

    @Override
    public void setI(double kI) {
        if (this.m_kI != kI) {
            m_controller.setI(kI);
            this.m_kI = kI;
        }
    }

    /**
     * gets the Differential coefficient
     *
     * @return differential coefficient
     */
    @Override
    public double getD() {
        return m_controller.getD();
    }

    @Override
    public void setD(double kD) {
        if (this.m_kD != kD) {
            m_controller.setD(kD);
            this.m_kD = kD;
        }
    }

    /**
     * gets IZone value of slot 0
     *
     * @return IZone value
     */

    public double getIzone() {
        return m_controller.getIZone();
    }

    public void setIzone(double kIz) {
        if (this.m_kIz != kIz) {
            m_controller.setIZone(kIz);
            this.m_kIz = kIz;
        }
    }

    /**
     * gets max I accumulator
     *
     * @return max I Accumulator
     */
    public double getIMaxAccum() {
        return m_controller.getIMaxAccum(0);
    }

    public void setIMaxAccum(double kIMaxAccum) {
        if (this.m_kIMaxAccum != kIMaxAccum) {
            m_controller.setIMaxAccum(kIMaxAccum, 0);
            this.m_kIMaxAccum = kIMaxAccum;
        }
    }

    /**
     * gets feed forward gain
     * @return Fees-forward Gain
     */
    public double getF() {
        return m_controller.getFF();
    }

    public void setF(double kF) {
        if (this.m_kF != kF) {
            m_controller.setFF(kF);
            this.m_kF = kF;
        }
    }

    /**
     * gets Max Output
     *
     * @return maximum output
     */
    public double getMaxOutput() {
        return m_controller.getOutputMax();
    }

    public double getMinOutput() {
        return m_controller.getOutputMin();
    }

    public void setPID(double p, double i,double d) {
        this.setP(p);
        this.setI(i);
        this.setD(d);
    }

    public void setPIDF(double p, double i, double d, double f) {
        this.setP(p);
        this.setI(i);
        this.setD(d);
        this.setF(f);
    }

    public void setOutputRange(double kMinOutput, double kMaxOutput) {
        if ((this.m_kMinOutput != kMinOutput) || (this.m_kMaxOutput != kMaxOutput)) {
            m_controller.setOutputRange(kMinOutput, kMaxOutput);
            this.m_kMinOutput = kMinOutput;
            this.m_kMaxOutput = kMaxOutput;
        }
    }

    /**
     *
     * @return the control type the PIDController is using (current, velocity, position)
     */
    public ControlType getControlType() { return this.m_controlType; }

    public void setControlType(ControlType controlType) {
        this.m_controlType = controlType;
    }

    @Override
    public void runPID() {
        m_controller.setReference(m_setpoint, m_controlType);
    }

    public void runPID(double target) {
        setSetpoint(target);
        runPID();
    }

    public CANPIDController getController() {
        return this.m_controller;
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
     */
    @Override
    public void setSetpoint(double setpoint) {
        this.m_setpoint = setpoint;
    }

    /**
     * gets setpoint
     *
     * @return setpoint
     */
    public double getSetpoint() {
        return this.m_setpoint;
    }

    /**
     * resets I Accum to 0
     */
    @Override
    public void reset() {
        this.m_controller.setIAccum(0);
    }

    /**
     * sets encoder to a given position
     *
     * @param position desired position
     */
    @Override
    public void setEncoderPosition(double position) {
        m_encoder.setPosition(position);
    }

    /**
     * get PID output based on control mode
     * @param controlType control mode
     * @return output
     */
    public double getOutput(ControlType controlType) {
        switch (controlType) {
            case kCurrent:
                return this.getOutputCurrent();
            case kSmartVelocity:
            case kVelocity:
                return m_encoder.getVelocity();
            case kSmartMotion:
            case kPosition:
                return m_encoder.getPosition();
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
    @Override
    public double getOutput()  {
        switch (m_controlType) {
            case kCurrent:
                return this.getOutputCurrent();
            case kSmartVelocity:
            case kVelocity:
                return m_encoder.getVelocity();
            case kSmartMotion:
            case kPosition:
                return m_encoder.getPosition();
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
