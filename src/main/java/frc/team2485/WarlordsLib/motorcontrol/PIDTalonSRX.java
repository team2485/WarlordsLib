package frc.team2485.WarlordsLib.motorcontrol;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.team2485.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.SavableConfigs;

public class PIDTalonSRX extends WL_TalonSRX {

    private ControlMode m_controlMode;

    private double m_setpoint;

    private double kP, kI, kD, kIz, kF, kMaxOutput, kMinOutput, kIMaxAccum;

    private int pidIdx = 0;

    /**
     * Constructor for TalonSRX object
     *
     * @param deviceNumber CAN Device ID of Device
     */
    public PIDTalonSRX(int deviceNumber, ControlMode m_controlMode) {
        super(deviceNumber);

        this.m_controlMode = m_controlMode;
    }

    public void setP(double kP) {
        if(this.kP != kP) {
            this.config_kP(pidIdx, kP);
            this.kP = kP;
        }
    }

    public double getP() {
        return this.kP;
    }

    public void setI(double kI) {
        if(this.kI != kI) {
            this.config_kI(pidIdx, kI);
            this.kI = kI;
        }
    }

    public double getI() {
        return this.kI;
    }

    public void setD(double kD) {
        if(this.kD != kD) {
            this.config_kD(pidIdx, kD);
            this.kD = kD;
        }
    }

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
            this.kIMaxAccum = kIMaxAccum;
        }
    }

    public double getIMaxAccum() {
        return this.kIMaxAccum;
    }

    public void setSetpoint(double setpoint) {
        this.m_setpoint = setpoint;
    }

    public double getSetpoint() {
        return this.m_setpoint;
    }

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
//        builder.setSmartDashboardType("PIDController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("f", this::getF, this::setF);
        builder.addDoubleProperty("iZone", this::getIzone, this::setIzone);
        builder.addDoubleProperty("iMaxAccum", this::getIMaxAccum, this::setIMaxAccum);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }

    public void loadConfigs(LoadableConfigs configs) {
        this.setP(configs.getDouble("p", this.getP()));
        this.setI(configs.getDouble("i", this.getI()));
        this.setD(configs.getDouble("d", this.getD()));
        this.setF(configs.getDouble("f", this.getF()));
        this.setIzone(configs.getDouble("iZone", this.getIzone()));
        this.setIMaxAccum(configs.getDouble("iMaxAccum", this.getIMaxAccum()));
    }

    public void saveConfigs(SavableConfigs configs) {
        configs.put("p", this.getP());
        configs.put("i", this.getI());
        configs.put("d", this.getD());
        configs.put("f", this.getF());
        configs.put("iZone", this.getIzone());
        configs.put("iMaxAccum", this.getIMaxAccum());
    }

}
