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


public class PIDTalonSRX extends WL_TalonSRX implements Configurable, PIDMotorController<ControlMode, FeedbackDevice> {

    private ControlMode m_controlMode;

    private double m_setpoint;

    private double kP, kI, kD, kIz, kF, kRR, kIMaxAccum;

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
        this.kP = this.getTerms().kP;
        this.kI = this.getTerms().kI;
        this.kD = this.getTerms().kD;
        this.kF = this.getTerms().kF;
        this.kIz = this.getTerms().integralZone;
        this.kIMaxAccum = this.getTerms().maxIntegralAccumulator;

        TalonSRXConfiguration configs = new TalonSRXConfiguration();
        this.baseGetAllConfigs(configs, 0);
        this.kRR = configs.closedloopRamp;


    }

    public SlotConfiguration getTerms() {
        SlotConfiguration terms = new SlotConfiguration();
        this.getSlotConfigs(terms);
        return terms;
    }

    @Override
    public void setP(double kP) {
        if(this.kP != kP) {
            this.config_kP(pidIdx, kP);
            this.kP = kP;
        }
    }

    @Override
    public double getP() {
        return this.kP;
    }

    @Override
    public void setI(double kI) {
        if(this.kI != kI) {
            this.config_kI(pidIdx, kI);
            this.kI = kI;
        }
    }

    @Override
    public double getI() {
        return this.kI;
    }

    @Override
    public void setD(double kD) {
        if(this.kD != kD) {
            this.config_kD(pidIdx, kD);
            this.kD = kD;
        }
    }

    @Override
    public double getD() {
        return this.kD;
    }

    public void setF(double kF) {
        if(this.kF != kF) {
            this.config_kF(pidIdx, kF);
            this.kF = kF;
        }
    }

    public double getF() {
        return this.kF;
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

    public void setIzone(double kIz) {
        if(this.kIz != kIz) {
            this.config_IntegralZone(pidIdx, (int) kIz);
            this.kIz = kIz;
        }
    }

    public double getIzone() {
        return this.kIz;
    }

    public void setIMaxAccum(double kIMaxAccum) {
        if(this.kIMaxAccum != kIMaxAccum) {
            this.configMaxIntegralAccumulator(pidIdx, kIMaxAccum);
        }

    }

    public double getIMaxAccum() {
        return this.kIMaxAccum;
    }

    public void setClosedLoopRampRate(double seconds) {
        if (this.kRR != seconds) {
            this.configClosedloopRamp(seconds);
            this.kRR = seconds;
        }

    }

    public double getClosedLoopRampRate() {
        return this.kRR;
    }

    @Override
    public void setSetpoint(double setpoint) {
        this.m_setpoint = setpoint;
    }

    @Override
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

    @Override
    public void runPID(double target) {
        this.setSetpoint(target);
        this.runPID();
    }

    @Override
    public ControlMode getControlMode() {
        return this.m_controlMode;
    }

    @Override
    public void setControlMode(ControlMode controlMode) {
        this.m_controlMode = controlMode;
    }

    @Override
    public void setFeedbackDeviceType(FeedbackDevice feedbackDevice) {
        this.configSelectedFeedbackSensor(feedbackDevice);
    }
    /**
     * resets PID controller.
     */
    @Override
    public void reset() {
        this.setIntegralAccumulator(0);
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
