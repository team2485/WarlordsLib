package frc.team2485.WarlordsLib.control.legacy;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.team2485.WarlordsLib.robotConfigs.Configurable;
import frc.team2485.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.SavableConfigs;

/**
 * PID Controller based mainly on Jeremy McCulloch's PIDController.
 * Takes some ideas from WPILib's CoupledPIDController.
 *
 * @author Jeremy McCulloch
 * @author Nathan Sariowan
 */
public class CoupledPIDController implements Configurable, Sendable {

    /**
     * Default period for WPILib is 20 milliseconds.
     */
    public static final double DEFAULT_PERIOD = 0.02;

    /**
     * Rate in seconds that the PID Controller runs at.
     */
    private double _period = DEFAULT_PERIOD;

    /**
     * Proportional Term
     */
    private double _kP=0;

    /**
     * Integral Term
     */
    private double _kI=0;

    /**
     * Derivative Term
     */
    private double _kD=0;


    /**
     * Feed Forward Term
     */
    private double _kF = 0;

    /**
     * Integral anti-windup constant; multiplied by saturation error.
     */
    private double m_kAW = 0;

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
    private double _lastOutput;

    private double _lastMeasurement;

    public CoupledPIDController(double period) {
        this._period = period;
    }

    public CoupledPIDController() {
        this._period = DEFAULT_PERIOD;
    }

    public void setCoupled(boolean coupled) {
        this._couplePID = true;
    }

    public boolean getCoupled() {
        return this._couplePID;
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
     * @param p proportional constant
     */
    public void setP(double p) {
        this._kP = p;
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
     * @param i integral constant
     */
    public void setI(double i) {
        this._kI = i;
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
     * @param d derivative constant
     */
    public void setD(double d) {
        this._kD = d;
    }

    /**
     * Returns feed forward constant
     * @return kF Feed forward Constant
     */
    public double getF() { return this._kF; }

    /**
     * Set feed forward constant
     * @param f
     */
    public void setF(double f) {
        this._kF = f;
    }

    /**
     * Get integral anti-windup constant
     * @return kAW constant
     */
    public double getAW() { return this.m_kAW; }

    /**
     * Set integral anti-windup constant, which is multiplied by the saturation error
     * @param aw Kt constant
     */
    public void setAW(double aw) {
        this.m_kAW = aw;
    }


    public void setPID(double p, double i, double d) {
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
            iTerm = _kI * (_totalError - m_kAW * _saturationError);
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

        double fTerm = _kF * _setpoint;

        double unclampedOutput = pTerm + iTerm + dTerm + fTerm + addTerm;

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

        this._lastOutput = output;

        return output;
    }

    public double calculate(double input) {
        return calculate(input, 0);
    }

    private double getOutput() {
        return _lastOutput;
    }

    private double getMeasurement() {
        return _lastMeasurement;
    }

    public void disable() {
        _totalError = 0;
        _integratedError = 0;
        _prevError = 0;
    }

    public void reset() {
        disable();
    }

    @Override
    public void initSendable(SendableBuilder builder) {

        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
        builder.setSafeState(this::disable);

        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("f", this::getF, this::setF);
        builder.addDoubleProperty("aw", this::getAW, this::setAW);
        builder.addDoubleProperty("output", this::getOutput, null);
    }

    @Override
    public void loadConfigs(LoadableConfigs configs) {
        setP(configs.getDouble("p", this.getP()));
        setI(configs.getDouble("i", this.getI()));
        setD(configs.getDouble("d", this.getD()));
        setF(configs.getDouble("f", this.getF()));
        setF(configs.getDouble("aw", this.getAW()));
    }

    @Override
    public void saveConfigs(SavableConfigs configs) {
        configs.put("p", this.getP());
        configs.put("i", this.getI());
        configs.put("d", this.getD());
        configs.put("f", this.getF());
        configs.put("aw", this.getP());
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
