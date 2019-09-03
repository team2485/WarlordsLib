package frc.team2485.WarlordsLib.controls;

import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

import java.util.ArrayList;
import java.util.Arrays;

public class WarlordsController extends SendableBase {

    public static double DEFAULT_PERIOD = 0.02;

    private double m_period;

    private double m_minInput;
    private double m_maxInput;

    private double m_inputRange;

    private double m_minOutput = -1.0;
    private double m_maxOutput = 1.0;

    private boolean m_isContinuous;

    private double m_setpoint;

    private double m_positionError;
    private double m_velocityError;
    private double m_prevError;

    private ArrayList<ControlTerm> m_controlTerms;

    public WarlordsController(ControlTerm... controlTerms) {
        m_period = DEFAULT_PERIOD;
        this.m_controlTerms = new ArrayList<ControlTerm>(Arrays.asList(controlTerms));
    }

    public double getPeriod() {
        return m_period;
    }

    public double getMinInput() {
        return m_minInput;
    }
    public double getMaxInput() {
        return m_maxInput;
    }

    public double getInputRange() {
        return m_inputRange;
    }

    public double getMinOutput() {
        return m_minOutput;
    }
    public double getMaxOutput() {
        return m_maxOutput;
    }

    public boolean isContinuous() {
        return m_isContinuous;
    }

    public void setSetpoint(double setpoint) {
        this.m_setpoint = setpoint;
    }

    public double getSetpoint() {
        return m_setpoint;
    }

    public double getPositionError() {
        return m_positionError;
    }

    public double getVelocityError() {
        return m_velocityError;
    }

    public double getPrevError() {
        return m_prevError;
    }

    public void resetTerms(ControlTerm... controlTerms) {
        m_controlTerms.clear();
        m_controlTerms.addAll(Arrays.asList(controlTerms));
    }

    public void calculate(double measurement, double setpoint) {
        setSetpoint(setpoint);
        calculate(measurement);
    }

    /**
     * The big kahuna.
     * @param measurement
     * @return the sum of all the calculated terms.
     */
    public double calculate(double measurement) {
        double sum = 0;

        m_prevError = m_positionError;
        m_positionError = getContinuousError(m_setpoint - measurement);
        m_velocityError = (m_positionError - m_prevError) / m_period;


        for (ControlTerm term : m_controlTerms) {
            sum += term.calculate(this);
        }

        return clamp(sum, m_minOutput, m_maxOutput);
    }

    public void reset() {
        for (ControlTerm term : m_controlTerms) {
            term.reset();
        }
        m_prevError = 0;
    }

    @Override
    public void initSendable(SendableBuilder builder) {

    }

    private double getContinuousError(double error) {
        if (m_isContinuous && m_inputRange > 0) {
            error %= m_inputRange;
            if (Math.abs(error) > m_inputRange / 2) {
                if (error > 0) {
                    return error - m_inputRange;
                } else {
                    return error + m_inputRange;
                }
            }
        }
        return error;
    }

    protected static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }
}
