package frc.team2485.WarlordsLib.control;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.team2485.WarlordsLib.robotConfigs.Configurable;
import frc.team2485.WarlordsLib.robotConfigs.ConfigurableBuilder;

public class WL_PIDController extends PIDController implements Configurable {

    private double m_Kf = 0;

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
        return super.calculate(measurement) + m_Kf * this.getSetpoint();
    }

    @Override
    public void initConfigurable(ConfigurableBuilder builder) {
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("f", this::getF, this::setF);
    }
}
