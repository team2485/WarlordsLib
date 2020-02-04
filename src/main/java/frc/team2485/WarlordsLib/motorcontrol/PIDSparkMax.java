package frc.team2485.WarlordsLib.motorcontrol;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.team2485.WarlordsLib.motorcontrol.base.PIDMotorController;
import frc.team2485.WarlordsLib.robotConfigs.Configurable;
import frc.team2485.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.SavableConfigs;

public class PIDSparkMax extends WL_SparkMax implements Configurable, PIDMotorController<ControlType, EncoderType> {

    private ControlType m_controlType;

    private CANPIDController m_controller;

    private CANEncoder m_encoder;

    private double m_setpoint;

    private double m_kP, m_kI, m_kD, m_kIz, m_kF, m_kMaxOutput, m_kMinOutput, m_kIMaxAccum;

    /**
     * Create a new Brushless SPARK MAX Controller
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

    /**
     * Set the PID's feedback device
     * @param feedbackDeviceType feedbackDevice
     */
    @Override
    public void setFeedbackDeviceType(EncoderType feedbackDeviceType) {
        m_controller.setFeedbackDevice(new CANEncoder(this, feedbackDeviceType, 0)); // the counts per rev is not used
    }

    @Override
    public ControlType getControlMode() {
        return this.m_controlType;
    }

    @Override
    public void setControlMode(ControlType controlType) {
        this.m_controlType = controlType;
    }

    /**
     * gets the proportional coefficient
     * @return proportional coefficient
     */
    @Override
    public double getP() {
        return m_controller.getP();
    }

    /**
     * sets proportional coefficient
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
     * @return integral coefficient
     */
    @Override
    public double getI() {
        return m_controller.getI();

    }

    /**
     * Set integral coefficient
     * @param kI integral coefficient
     */
    @Override
    public void setI(double kI) {
        if (this.m_kI != kI) {
            m_controller.setI(kI);
            this.m_kI = kI;
        }
    }

    /**
     * gets the Differential coefficient
     * @return differential coefficient
     */
    @Override
    public double getD() {
        return m_controller.getD();
    }

    /**
     * Set differential coefficient
     * @param kD differential coefficient
     */
    @Override
    public void setD(double kD) {
        if (this.m_kD != kD) {
            m_controller.setD(kD);
            this.m_kD = kD;
        }
    }

    /**
     * gets IZone value of slot 0. The max distance from target where I term will be calculated
     * @return IZone value
     */
    public double getIzone() {
        return m_controller.getIZone();
    }

    /**
     * Set max distance from target where I term will be calculated
     * @param kIz IZone value
     */
    public void setIzone(double kIz) {
        if (this.m_kIz != kIz) {
            m_controller.setIZone(kIz);
            this.m_kIz = kIz;
        }
    }

    /**
     * gets max I accumulator. Constrains I accumulator to prevent integral windup.
     * @return max I Accumulator
     */
    public double getIMaxAccum() {
        return m_controller.getIMaxAccum(0);
    }

    /**
     * Sets max I accumulator. Constrains I accumulator to prevent integral windup.
     * @param kIMaxAccum max I Accumulator
     */
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

    /**
     * Sets feed forward gain
     * @param kF feed forward gain
     */
    public void setF(double kF) {
        if (this.m_kF != kF) {
            m_controller.setFF(kF);
            this.m_kF = kF;
        }
    }

    /**
     * gets max output
     * @return maximum output
     */
    public double getMaxOutput() {
        return m_controller.getOutputMax();
    }

    /**
     * gets min output
     * @return minimum output
     */
    public double getMinOutput() {
        return m_controller.getOutputMin();
    }

    /**
     * Set P, I and D constants
     * @param p proportional coefficient
     * @param i integral coefficient
     * @param d derivative coefficient
     */
    public void setPID(double p, double i,double d) {
        this.setP(p);
        this.setI(i);
        this.setD(d);
    }

    /**
     * Set P, I, D, and F constants
     * @param p proportional coefficient
     * @param i integral coefficient
     * @param d derivative coefficient
     * @param f feed forward coefficient
     */
    public void setPIDF(double p, double i, double d, double f) {
        this.setP(p);
        this.setI(i);
        this.setD(d);
        this.setF(f);
    }

    /**
     * Set min and max output of PID
     * @param kMinOutput min output
     * @param kMaxOutput max output
     */
    public void setOutputRange(double kMinOutput, double kMaxOutput) {
        if ((this.m_kMinOutput != kMinOutput) || (this.m_kMaxOutput != kMaxOutput)) {
            m_controller.setOutputRange(kMinOutput, kMaxOutput);
            this.m_kMinOutput = kMinOutput;
            this.m_kMaxOutput = kMaxOutput;
        }
    }

    /**
     * Run PID on this controller from setpoint
     */
    @Override
    public void runPID() {
        m_controller.setReference(m_setpoint, m_controlType);
    }

    /**
     * Set setpoint and run PID on this controller
     * @param target
     */
    @Override
    public void runPID(double target) {
        setSetpoint(target);
        runPID();
    }

    /**
     * Set the controller reference value based on the selected control mode.
     * @param setpoint The value to set depending on the control mode.
     */
    @Override
    public void setSetpoint(double setpoint) {
        this.m_setpoint = setpoint;
    }

    /**
     * get setpoint
     * @return setpoint
     */
    public double getSetpoint() {
        return this.m_setpoint;
    }

    /**
     * resets PID controller.
     */
    @Override
    public void resetPID() {
        this.m_controller.setIAccum(0);
    }

    /**
     * resets encoder to a given position
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
    public double getSensorOutput()  {
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

    public CANPIDController getController() {
        return m_controller;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
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
        this.setClosedLoopRampRate(configs.getDouble("rampRate", this.getClosedLoopRampRate()));
    }

    @Override
    public void saveConfigs(SavableConfigs configs) {
        configs.put("p", this.getP());
        configs.put("i", this.getI());
        configs.put("d", this.getD());
        configs.put("f", this.getF());
        configs.put("iZone", this.getIzone());
        configs.put("iMaxAccum", this.getIMaxAccum());
        configs.put("rampRate", this.getClosedLoopRampRate());
    }
}
