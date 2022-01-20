package frc.team2485.WarlordsLib.motorcontrol;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.team2485.WarlordsLib.motorcontrol.base.PIDMotorController;
import frc.team2485.WarlordsLib.robotConfigs.Configurable;
import frc.team2485.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.SavableConfigs;

public class PIDTalonSRX extends WL_TalonSRX
    implements Configurable, PIDMotorController<ControlMode, FeedbackDevice> {

  private ControlMode m_controlMode;

  private double m_setpoint;

  private double kP, kI, kD, kIz, kF, kRR, kIMaxAccum;

  private int pidIdx = 0;

  private double m_conversionFactor = 1;

  private double m_threshold;

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

  private SlotConfiguration getTerms() {
    SlotConfiguration terms = new SlotConfiguration();
    this.getSlotConfigs(terms);
    return terms;
  }

  /**
   * Set Proportional constant
   *
   * @param kP proportional constant
   */
  @Override
  public void setP(double kP) {
    if (this.kP != kP) {
      this.config_kP(pidIdx, kP);
      this.kP = kP;
    }
  }

  /**
   * Set proportional constant
   *
   * @return proportional constant
   */
  @Override
  public double getP() {
    return this.kP;
  }

  /**
   * Get integral constant
   *
   * @param kI integral constant
   */
  @Override
  public void setI(double kI) {
    if (this.kI != kI) {
      this.config_kI(pidIdx, kI);
      this.kI = kI;
    }
  }

  /**
   * Get integral constant
   *
   * @return integral constant
   */
  @Override
  public double getI() {
    return this.kI;
  }

  /**
   * Set derivative constant
   *
   * @param kD derivative constant
   */
  @Override
  public void setD(double kD) {
    if (this.kD != kD) {
      this.config_kD(pidIdx, kD);
      this.kD = kD;
    }
  }

  /**
   * Get derivative constant
   *
   * @return derivative constant
   */
  @Override
  public double getD() {
    return this.kD;
  }

  /**
   * Set feed forward constant
   *
   * @param kF feed forward constant
   */
  public void setF(double kF) {
    if (this.kF != kF) {
      this.config_kF(pidIdx, kF);
      this.kF = kF;
    }
  }

  /**
   * Get feed forward constant
   *
   * @return feed forward constant
   */
  public double getF() {
    return this.kF;
  }

  /**
   * Set P, I and D constants
   *
   * @param p proportional coefficient
   * @param i integral coefficient
   * @param d derivative coefficient
   */
  public void setPID(double p, double i, double d) {
    this.setP(p);
    this.setI(i);
    this.setD(d);
  }

  /**
   * Set P, I, D, and F constants
   *
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
    if (this.kIz != kIz) {
      this.config_IntegralZone(pidIdx, (int) kIz);
      this.kIz = kIz;
    }
  }

  public double getIzone() {
    return this.kIz;
  }

  public void setIMaxAccum(double kIMaxAccum) {
    if (this.kIMaxAccum != kIMaxAccum) {
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

  /**
   * Controller will multiply the setpoint by this number.
   *
   * @param setpointConversionFactor conversion factor
   */
  public void setDistancePerPulse(double setpointConversionFactor) {
    this.m_conversionFactor = setpointConversionFactor;
  }

  public double getConversionFactor() {
    return this.m_conversionFactor;
  }

  public double getClosedLoopRampRate() {
    return this.kRR;
  }

  @Override
  public void setSetpoint(double setpoint) {
    this.m_setpoint = (setpoint) / m_conversionFactor;
  }

  @Override
  public double getSetpoint() {
    return this.m_setpoint * m_conversionFactor;
  }

  /** Run PID on this controller from setpoint */
  @Override
  public void runPID() {
    this.set(m_controlMode, m_setpoint);
  }

  @Override
  public void runPID(double target) {
    this.setSetpoint(target);
    this.runPID();
  }

  /**
   * Get the control mode of this pid
   *
   * @return control mode of this pid
   */
  @Override
  public ControlMode getControlMode() {
    return this.m_controlMode;
  }

  /**
   * Set the control mode of this pid
   *
   * @param controlMode control mode of the pid
   */
  @Override
  public void setControlMode(ControlMode controlMode) {
    this.m_controlMode = controlMode;
  }

  /**
   * Set the feedback device type for the pid
   *
   * @param feedbackDevice type of sensor
   */
  @Override
  public void setFeedbackDeviceType(FeedbackDevice feedbackDevice) {
    this.configSelectedFeedbackSensor(feedbackDevice);
  }

  /** resets PID controller. */
  @Override
  public void resetPID() {
    this.setIntegralAccumulator(0);
  }

  /**
   * Resets encoder to a given position in radians.
   *
   * @param position desired position
   */
  public void setEncoderPosition(double position) {
    this.setSelectedSensorPosition((int) (position / m_conversionFactor));
  }

  /**
   * Get selected encoder position adjusted for conversion factor and offset
   *
   * @return encoder position
   */
  public double getEncoderPosition() {
    return (this.getSelectedSensorPosition() * m_conversionFactor);
  }

  /**
   * Get selected sensor velocity adjusted for conversion factor
   *
   * @return connected encoder velocity
   */
  public double getEncoderVelocity() {
    return this.getSelectedSensorVelocity() * m_conversionFactor;
  }

  /**
   * Set threshold for atTarget()
   *
   * @param tolerance pid is at target if within sensor output +/- this value
   */
  public void setTolerance(double tolerance) {
    this.m_threshold = Math.abs(tolerance);
  }

  /**
   * Get threshold for controller
   *
   * @return threshold
   */
  public double getThreshold() {
    return this.m_threshold;
  }

  /**
   * Returns true if sensor output is within a given threshold
   *
   * @return true when sensor is within threshold
   */
  public boolean atTarget() {
    return atTarget(this.m_threshold);
  }

  /**
   * Returns true if sensor output is within a given threshold
   *
   * @param threshold pid is at target if within sensor output +/- this value
   * @return true when sensor is within threshold
   */
  public boolean atTarget(double threshold) {
    return Math.abs(this.getSensorOutput() - this.getSetpoint()) < threshold;
  }

  /**
   * Get sensor output based on control mode
   *
   * @return Current: supply current; Velocity: encoder velocity; Position: encoder position;
   *     Percent output: motor output percent;
   */
  public double getSensorOutput() {
    switch (m_controlMode) {
      case Current:
        return this.getSupplyCurrent();
      case Velocity:
        return this.getEncoderVelocity();
      case Position:
        return this.getEncoderPosition();
      case PercentOutput:
        return this.getMotorOutputPercent();
      default:
        return 0;
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType("PIDController");
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
