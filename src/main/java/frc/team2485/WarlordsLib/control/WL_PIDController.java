package frc.team2485.WarlordsLib.control;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.team2485.WarlordsLib.robotConfigs.Configurable;
import frc.team2485.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.SavableConfigs;

/**
 * PID controller with:
 *      Configurable implementation
 *      Feedforward term
 *      Min and max output
 */
public class WL_PIDController extends PIDController implements Configurable {

    private double m_Kf = 0;

    private double m_minOutput = 0;
    private double m_maxOutput = 0;

    public WL_PIDController() {
        super(0,0,0);
    }

    /**
     * Get feedforward coefficient
     * @return Feedforward coefficient
     */
    public double getF() {
        return m_Kf;
    }

    /**
     * Set feedforward coefficient
     * @param Kf Feedforward coefficient
     */
    public void setF(double Kf) {
        this.m_Kf = Kf;
    }

    /**
     * Sets the PID Controller gain parameters.
     *
     * <p>Set the proportional, integral, and differential coefficients.
     *
     * @param Kp The proportional coefficient.
     * @param Ki The integral coefficient.
     * @param Kd The derivative coefficient.
     * @param Kf The feedforward coefficient
     */
    public void setPIDF(double Kp, double Ki, double Kd, double Kf) {
        this.setPID(Kp, Ki, Kd);
        this.m_Kf = Kf;
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     */
    @Override
    public double calculate(double measurement) {
        double output = super.calculate(measurement) + m_Kf * this.getSetpoint();

        if (m_minOutput != m_maxOutput) {
            return MathUtil.clamp(output, m_minOutput, m_maxOutput);
        }

        return output;
    }

    /**
     * Set output range of controller
     * @param minOutput min output of controller
     * @param maxOutput max output of controller
     */
    public void setOutputRange(double minOutput, double maxOutput) {
        this.m_minOutput = minOutput;
        this.m_maxOutput = maxOutput;
    }

    /**
     * Get min output of controller
     * @return min output
     */
    public double getMinOutput() {
        return this.m_minOutput;
    }

    /**
     * Get max output of controller
     * @return max output
     */
    public double getMaxOutput() {
        return this.m_maxOutput;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("f", this::getF, this::setF);
    }

    @Override
    public void loadConfigs(LoadableConfigs configs) {
        setP(configs.getDouble("p", getP()));
        setI(configs.getDouble("i", getI()));
        setD(configs.getDouble("d", getD()));
        setF(configs.getDouble("f", getF()));
    }

    @Override
    public void saveConfigs(SavableConfigs configs) {
        configs.put("p", getP());
        configs.put("i", getI());
        configs.put("d", getD());
        configs.put("f", getF());
    }
}
