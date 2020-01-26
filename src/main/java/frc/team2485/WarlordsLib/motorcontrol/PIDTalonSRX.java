package frc.team2485.WarlordsLib.motorcontrol;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.BaseMotorControllerConfiguration;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.team2485.WarlordsLib.motorcontrol.base.PIDMotorController;
import frc.team2485.WarlordsLib.robotConfigs.Configurable;
import frc.team2485.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.SavableConfigs;
import frc.team2485.WarlordsLib.sensors.TalonSRXEncoder;

import static frc.team2485.WarlordsLib.sensors.TalonSRXEncoder.TalonSRXEncoderType.*;


public class PIDTalonSRX extends WL_TalonSRX implements Configurable, PIDMotorController {

    private ControlMode m_controlMode;

    private double m_setpoint;

    private double kP, kI, kD, kIz, kF, kMaxOutput, kMinOutput, kIMaxAccum;

    private int pidIdx = 0;

    private TalonSRXEncoder m_encoder;

    /**
     * Constructor for TalonSRX object
     *
     * @param deviceID CAN Device ID of Device
     */
    public PIDTalonSRX(int deviceID, ControlMode controlMode) {
        super(deviceID);

        this.m_controlMode = controlMode;

    }

    public SlotConfiguration getTerms() {
        SlotConfiguration terms = new SlotConfiguration();
        this.getSlotConfigs(terms);
        return terms;
    }

    public void setP(double kP) {
        this.config_kP(pidIdx, kP);
    }

    public double getP() {
        return this.getTerms().kP;
    }

    public void setI(double kI) {
        this.config_kP(pidIdx, kI);
    }

    public double getI() {
        return this.getTerms().kI;
    }

    public void setD(double kD) {
        this.config_kD(pidIdx, kD);
    }

    public double getD() {
        return this.getTerms().kD;
    }

    public void setF(double kF) {
        this.config_kF(pidIdx, kF);
    }

    public double getF() {
        return this.getTerms().kF;
    }

    public void setIzone(double kIz) {
        this.config_IntegralZone(pidIdx, (int) kIz);
    }

    public double getIzone() {
        return this.getTerms().integralZone;
    }

    public void setIMaxAccum(double kIMaxAccum) {
        this.configMaxIntegralAccumulator(pidIdx, kIMaxAccum);
    }

    public double getIMaxAccum() {
        return this.getTerms().maxIntegralAccumulator;
    }

    public void setClosedLoopRampRate(double seconds) {
        this.configClosedloopRamp(seconds);
    }

    public double getClosedLoopRampRate() {
        TalonSRXConfiguration configs = new TalonSRXConfiguration();
        this.baseGetAllConfigs(configs, 0);
        return configs.closedloopRamp;
    }

    public void setSetpoint(double setpoint) {
        this.m_setpoint = setpoint;
    }

    public double getSetpoint() {
        return this.m_setpoint;
    }

    /**
     * Run PID on this controller from setpoint
     */
    @Override
    public void runPID() {
        this.set(m_controlMode, m_setpoint);
    }

    public void runPID(double target) {
        this.setSetpoint(target);
        this.runPID();
    }

    public ControlMode getControlMode() {
        return this.m_controlMode;
    }

    public void setControlMode(ControlMode controlMode) {
        this.m_controlMode = controlMode;
    }


    public void configureFeedbackDevice(TalonSRXEncoder encoder) {
        this.m_encoder = encoder;
        FeedbackDevice feedbackDevice = FeedbackDevice.None;
        switch (encoder.getEncoderType()) {
            case ABSOLUTE:
                feedbackDevice = FeedbackDevice.PulseWidthEncodedPosition;
                break;
            case QUADRATURE:
                feedbackDevice = FeedbackDevice.QuadEncoder;
                break;
            case ANALOG:
                feedbackDevice = FeedbackDevice.Analog;
                break;
        }

        this.configSelectedFeedbackSensor(feedbackDevice);
    }
    /**
     * resets PID controller.
     * cannot find set I accumulator method so sets max to zero and then back
     */
    @Override
    public void reset() {
        double maxAccum = this.getIMaxAccum();
        this.setIMaxAccum(0);
        this.setIMaxAccum(maxAccum);
    }

    /**
     * resets encoder to a given position in radians
     * @param position desired position
     */
    public void setEncoderPosition(double position) {
        m_encoder.resetPosition(position);
    }

    public double getOutput() {
        switch (m_controlMode) {
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
        this.setClosedLoopRampRate((configs.getDouble("rampRate", this.getClosedLoopRampRate())));
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
