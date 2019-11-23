package frc.team2485.WarlordsLib.control;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * PID Controller based mainly on Jeremy McCulloch's PIDController.
 * Takes some ideas from WPILib's WL_PIDController.
 *
 * @author Jeremy McCulloch
 * @author Nathan Sariowan
 */
public class WarlordsPIDController extends SendableBase implements Controller {

    /**
     * Default period for WPILib is 20 milliseconds.
     */
    public static final double DEFAULT_PERIOD = 0.02;

    /**
     * Rate in seconds that the PID Controller runs at.
     */
    private double _period;

    /**
     * Proportional Term
     */
    private double _kP;

    /**
     * Integral Term
     */
    private double _kI;

    /**
     * Derivative Term
     */
    private double _kD;

    
    /**
     * Feed Forward Term
     */
    private double _kF = 0;

    /**
     * Integral anti-windup constant; multiplied by saturation error.
     */
    private double m_kT = 0;

    /**
     * Set true if the max value of the system is equal to the min (endpoints wrap around)
     */
    private boolean _continuous;

    /**
     * Min measurement input
     */
    private double _minInput;

    /**
     * Max measurement input
     */
    private double _maxInput;

    /**
     * Difference between max input and min input
     */
    private double _inputRange;

    private double _minOutput = -1;
    private double _maxOutput = 1;

    public enum Tolerance {
        kAbsolute, kPercent
    }

    private Tolerance _toleranceType = Tolerance.kAbsolute;

    private double _tolerance = 0.0;

    private double _setpoint;

    private double _error;

    /**
     * The sum of all the error, integrated selectively to avoid I term windup (see calculate method)
     */
    private double _integratedError;

    /**
     * Sum of all error
     */
    private double _totalError;

    /**
     * Error calculated at previous calculate call
     */
    private double _prevError;

    /**
     * Error calculated at previous calculate call
     */
    private double _saturationError;

    /**
     * True if output is at _minOutput
     */
    private boolean _atMinOutput;

    /**
     * True if output is at _maxOutput
     */
    private boolean _atMaxOutput;

    /**
     * Couples P term with I and D if true
     */
    private boolean _couplePID = true;

    /**
     * Last recorded output
     */
    private double _output;

    public WarlordsPIDController() {
        setPID(0,0,0);
        this._period = DEFAULT_PERIOD;
    }

    public WarlordsPIDController(double P, double I, double D) {
        setPID(P, I, D);
        this._period = DEFAULT_PERIOD;
    }

    public WarlordsPIDController(double P, double I, double D, double period) {
        setPID(P, I, D);
        _period = period;
    }

    /**
     * @return milliseconds between each loop
     */
    public double getPeriod() {
        return _period;
    }

    /**
     * Get proportional term.
     * @return kP proportional constant
     */
    public double getP() {
        return this._kP;
    }

    /**
     * Set proportional constant
     * @param P proportional constant
     */
    public void setP(double P) {
        this._kP = P;
    }

    /**
     * Get integral constant
     * @return kI integral constant
     */
    public double getI() {
        return this._kI;
    }

    /**
     * Set integral constant
     * @param I integral constant
     */
    public void setI(double I) {
        this._kI = I;
    }

    /**
     * Get derivative constant
     * @return kD derivative constant
     */
    public double getD() {
        return this._kD;
    }

    /**
     * Set derivative constant
     * @param D derivative constant
     */
    public void setD(double D) {
        this._kP = D;
    }

    /**
     * Returns feed forward constant
     * @return kF Feed forward Constant
     */
    public double getF() { return this._kF; }

    /**
     * Set feed forward constant
     * @param F
     */
    public void setF(double F) { this._kF = F; }

    /**
     * Get integral anti-windup constant
     * @return Kt constant
     */
    public double getT() { return this.m_kT; }

    /**
     * Set integral anti-windup constant, which is multiplied by the saturation error
     * @param T Kt constant
     */
    public void setT(double T) { this. m_kT = T; }


    public void setPID(double P, double I, double D) {
        this._kP = P;
        this._kI = I;
        this._kD = D;
    }

    public void setPIDF(double P, double I, double D, double F) {
        this._kP = P;
        this._kI = I;
        this._kD = D;
        this._kF = F;
    }

    public void setCoupled(double coupled) {
        this._couplePID = true;
    }

    public boolean isCoupledPID() {
        return this._couplePID;
    }

    /**
     * Set minimum and maximum values of controller output
     * @param minOutput
     * @param maxOutput
     */
    public void setOutputRange(double minOutput, double maxOutput) {
        this._minOutput = minOutput;
        this._maxOutput = maxOutput;
    }

    public double getInputRange() {
        return _inputRange;
    }

    /**
     * Set minimum and maximum values expected from input.
     * @param minInput
     * @param maxInput
     */
    public void setInputRange(double minInput, double maxInput) {
        this._minInput = minInput;
        this._maxInput = maxInput;

        this._inputRange = maxInput - minInput;

        if (_maxInput > _minInput) {
            _setpoint = Math.max(_minInput, Math.min(_setpoint, _maxInput));
        }

    }

    public double getSetpoint() {
        return _setpoint;
    }

    public void setSetpoint(double setpoint) {
        if (_maxInput > _minInput) {
            _setpoint = Math.max(_minInput, Math.min(setpoint, _maxInput));
        } else {
            _setpoint = setpoint;
        }
    }

    public double getError() {
        return _error;
    }

    public void enableContinuousInput(double minInput, double maxInput) {
        _continuous = true;
        setInputRange(minInput, maxInput);
    }

    public void disableContinuousInput() {
        _continuous = false;
    }

    public boolean isContinuous() {
        return this._continuous;
    }

    public void setAbsoluteTolerance(double absoluteTolerance) {
        this._toleranceType = Tolerance.kAbsolute;
        this._tolerance = absoluteTolerance;
    }

    public void setPercentTolerance(double percentTolerance) {
        this._toleranceType = Tolerance.kPercent;
        this._tolerance = percentTolerance;
    }

    public boolean atSetpoint() {
        return atSetpoint(_tolerance, _toleranceType);
    }

    public boolean atSetpoint(double tolerance, Tolerance toleranceType) {
        if (toleranceType == Tolerance.kPercent) {
            return Math.abs(_error) < _setpoint * tolerance;
        } else  {
            return Math.abs(_error) < tolerance;
        }
    }

    public double calculate(double measurement, double addTerm) {
        _prevError = _error;
        _error = getContinuousError(_setpoint - measurement);
        _totalError += _error * _period;

        double pTerm = _kP * _error;

        double iTerm;

        // Anti-windup logic: reduce integral error by a constant if saturating
        if (_atMaxOutput || _atMinOutput) {
            iTerm = _kI * (_totalError - m_kT * _saturationError);
        } else {
            //A temp integrated error for other anti-windup logic schemes in the future
            _integratedError += _error * _period;
            iTerm = _kI * _totalError;
        }

        if (_couplePID) {
            iTerm *= _kP;
        }

        double dTerm = _kD * ((_error - _prevError) / _period);

        if (_couplePID) {
            dTerm *= _kP;
        }

        double unclampedOutput = pTerm + iTerm + dTerm + addTerm;

        double output = 0;

        // Clamp output
        if (unclampedOutput < _minOutput) {
            output = _minOutput;
            _atMinOutput = true;
            _atMaxOutput = false;
        } else if (unclampedOutput > _maxOutput) {
            output = _maxOutput;
            _atMinOutput = false;
            _atMaxOutput = true;
        } else {
            output = unclampedOutput;
            _atMinOutput = false;
            _atMaxOutput = false;
        }

        _saturationError = unclampedOutput - output;

        return output;
    }

    private double getOutput() {
        return _output;
    }

    @Override
    public double calculate(double input) {
        return calculate(input, 0);
    }

    @Override
    public void disable() {
        _totalError = 0;
        _integratedError = 0;
        _prevError = 0;
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("WL_PIDController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("f", this::getF, this::setF);
        builder.addDoubleProperty("t", this::getT, this::setT);

        builder.addDoubleProperty("output", this::getOutput, null);
        builder.setSafeState(this::disable);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }

    /**
     * Gets difference of measurement and setpoint.
     * Calculates continuous error if continuous.
     * @return
     */
    public double getContinuousError(double error) {
        if (_continuous && _inputRange > 0) {
            while (Math.abs(error) > (_maxInput - _minInput) / 2) {
                if (error > 0) {
                    error -= _maxInput - _minInput;
                } else {
                    error += _maxInput - _minInput;
                }

            }
        }

        return error;
    }
}
