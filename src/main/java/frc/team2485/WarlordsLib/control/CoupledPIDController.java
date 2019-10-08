package frc.team2485.WarlordsLib.control;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * PID Controller based mainly on Jeremy McCulloch's PIDController.
 * Takes some ideas from WPILib's WL_PIDController.
 *
 * @author Jeremy McCulloch
 */
public class CoupledPIDController extends SendableBase implements Controller {

    /**
     * Default period for WPILib is 20 milliseconds.
     */
    public static final double DEFAULT_PERIOD = 0.02;

    // Rate in seconds that the PID Controller runs at.
    private double m_period;

    // Proportional Term
    private double m_Kp;

    // Integral Term
    private double m_Ki;

    // Derivative Term
    private double m_Kd;

    // Feed Forward Term
    private double m_Kf = 0;

    // Integral anti-windup constant; multiplied by saturation error.
    private double m_Kt = 0;

    // Set true if the max value of the system is equal to the min (endpoints wrap around)
    private boolean m_continuous;

    private double m_minInput;
    private double m_maxInput;

    // Difference between max input and min input
    private double m_inputRange;

    private double m_minOutput = -1;
    private double m_maxOutput = 1;

    public enum Tolerance {
        kAbsolute, kPercent
    }

    private Tolerance m_toleranceType = Tolerance.kAbsolute;

    private double m_tolerance = 0.0;

    private double m_setpoint;

    private double m_error;

    // The sum of all the error, integrated selectively to avoid I term windup (see calculate method)
    private double m_integratedError;

    // Sum of all error
    private double m_totalError;

    // Error calculated at previous calculate call
    private double m_prevError;

    // The difference between the saturated (clamped) output and the unsaturated output.
    private double m_saturationError;

    // True if output is at m_minOutput
    private boolean m_atMinOutput;

    // True if output is at m_maxOutput
    private boolean m_atMaxOutput;

    private double m_output;

    public CoupledPIDController(double P, double I, double D) {
        setPID(P, I, D);
        this.m_period = DEFAULT_PERIOD;
    }


    public CoupledPIDController(double P, double I, double D, double period) {
        setPID(P, I, D);
        m_period = period;
    }

    public double getPeriod() {
        return m_period;
    }

    /**
     * Get proportional term.
     * @return kP proportional constant
     */
    public double getP() {
        return this.m_Kp;
    }

    /**
     * Set proportional constant
     * @param P proportional constant
     */
    public void setP(double P) {
        this.m_Kp = P;
    }

    /**
     * Get integral constant
     * @return kI integral constant
     */
    public double getI() {
        return this.m_Ki;
    }

    /**
     * Set integral constant
     * @param I integral constant
     */
    public void setI(double I) {
        this.m_Ki = I;
    }

    /**
     * Get derivative constant
     * @return kD derivative constant
     */
    public double getD() {
        return this.m_Kd;
    }

    /**
     * Set derivative constant
     * @param D derivative constant
     */
    public void setD(double D) {
        this.m_Kp = D;
    }

    /**
     * Returns feed forward constant
     * @return kF Feed forward Constant
     */
    public double getF() { return this.m_Kf; }

    /**
     * Set feed forward constant
     * @param F
     */
    public void setF(double F) { this. m_Kf = F; }

    /**
     * Get integral anti-windup constant
     * @return Kt constant
     */
    public double getT() { return this.m_Kt; }

    /**
     * Set integral anti-windup constant, which is multiplied by the saturation error
     * @param T Kt constant
     */
    public void setT(double T) { this. m_Kt = T; }


    public void setPID(double P, double I, double D) {
        this.m_Kp = P;
        this.m_Ki = I;
        this.m_Kd = D;
    }

    public void setPIDF(double P, double I, double D, double F) {
        this.m_Kp = P;
        this.m_Ki = I;
        this.m_Kd = D;
        this.m_Kf = F;
    }

    /**
     * Set minimum and maximum values of controller output
     * @param minOutput
     * @param maxOutput
     */
    public void setOutputRange(double minOutput, double maxOutput) {
        this.m_minOutput = minOutput;
        this.m_maxOutput = maxOutput;
    }

    public double getInputRange() {
        return m_inputRange;
    }

    /**
     * Set minimum and maximum values expected from input.
     * @param minInput
     * @param maxInput
     */
    public void setInputRange(double minInput, double maxInput) {
        this.m_minInput = minInput;
        this.m_maxInput = maxInput;

        this.m_inputRange = maxInput - minInput;

        if (m_maxInput > m_minInput) {
            m_setpoint = Math.max(m_minInput, Math.min(m_setpoint, m_maxInput));
        }

    }

    public double getSetpoint() {
        return m_setpoint;
    }

    public void setSetpoint(double setpoint) {
        if (m_maxInput > m_minInput) {
            m_setpoint = Math.max(m_minInput, Math.min(setpoint, m_maxInput));
        } else {
            m_setpoint = setpoint;
        }
    }

    public double getError() {
        return m_error;
    }

    public void enableContinuousInput(double minInput, double maxInput) {
        m_continuous = true;
        setInputRange(minInput, maxInput);
    }

    public void disableContinuousInput() {
        m_continuous = false;
    }

    public boolean isContinuous() {
        return this.m_continuous;
    }

    public void setAbsoluteTolerance(double absoluteTolerance) {
        this.m_toleranceType = Tolerance.kAbsolute;
        this.m_tolerance = absoluteTolerance;
    }

    public void setPercentTolerance(double percentTolerance) {
        this.m_toleranceType = Tolerance.kPercent;
        this.m_tolerance = percentTolerance;
    }

    public boolean atSetpoint() {
        return atSetpoint(m_tolerance, m_toleranceType);
    }

    public boolean atSetpoint(double tolerance, Tolerance toleranceType) {
        if (toleranceType == Tolerance.kPercent) {
            return Math.abs(m_error) < m_setpoint * tolerance;
        } else  {
            return Math.abs(m_error) < tolerance;
        }
    }

    public double calculate(double measurement, double addTerm) {
        m_prevError = m_error;
        m_error = getContinuousError(m_setpoint - measurement);
        m_totalError += m_error * m_period;

        double pTerm = m_Kp * m_error;

        double iTerm;

        // Anti-windup logic: reduce integral error by a constant
        if (m_atMaxOutput || m_atMinOutput) {
            iTerm = m_Ki * (m_totalError - m_Kt * m_saturationError);
        } else {
            //A temp integrated error for other anti-windup logic schemes to come
            m_integratedError += m_error * m_period;
            iTerm = m_Ki * m_totalError;
        }

        double dTerm = m_Kd * m_Kp * ((m_error - m_prevError) / m_period);

        double unclampedOutput = pTerm + iTerm + dTerm + addTerm;

        double output = 0;

        // Clamp output
        if (unclampedOutput < m_minOutput) {
            output = m_minOutput;
            m_atMinOutput = true;
            m_atMaxOutput = false;
        } else if (unclampedOutput > m_maxOutput) {
            output = m_maxOutput;
            m_atMinOutput = false;
            m_atMaxOutput = true;
        } else {
            output = unclampedOutput;
            m_atMinOutput = false;
            m_atMaxOutput = false;
        }

        m_saturationError = unclampedOutput - output;

        return output;
    }

    private double getOutput() {
        return m_output;
    }

    @Override
    public double calculate(double input) {
        return calculate(input, 0);
    }

    @Override
    public void disable() {
        m_totalError = 0;
        m_integratedError = 0;
        m_prevError = 0;
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("WL_PIDController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("f", this::getF, this::setF);
        builder.addDoubleProperty("t", this::getT, this::setT);
        builder.setSafeState(this::disable);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);

    }

    /**
     * Gets difference of measurement and setpoint.
     * Calculates continuous error if continuous.
     * @return
     */
    public double getContinuousError(double error) {
        if (m_continuous && m_inputRange > 0) {
            while (Math.abs(error) > (m_maxInput - m_minInput) / 2) {
                if (error > 0) {
                    error -= m_maxInput - m_minInput;
                } else {
                    error += m_maxInput - m_minInput;
                }

            }
        }

        return error;
    }
}
