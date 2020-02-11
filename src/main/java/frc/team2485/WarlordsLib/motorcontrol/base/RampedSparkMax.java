package frc.team2485.WarlordsLib.motorcontrol.base;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RampedSparkMax extends CANSparkMax {

    private static double m_rampRate = 0.1;

    private static double m_velocityThreshold = 500; //rpm

    private double m_lastValue;

    /**
     * Create a new SPARK MAX Controller
     *
     * @param deviceID The device ID.
     * @param type     The motor type connected to the controller. Brushless motors
     *                 must be connected to their matching color and the hall sensor
     *                 plugged in. Brushed motors must be connected to the Red and
     */
    public RampedSparkMax(int deviceID, MotorType type) {
        super(deviceID, type);

        m_lastValue = 0;
    }

    @Override
    public void set(double speed) {
        double desired = speed;
        double output = speed;

        double currSpeed = this.getEncoder().getVelocity();

//        if ((currSpeed > m_velocityThreshold && desired < 0)
//                || (currSpeed < -m_velocityThreshold && desired > 0)) {
//            output = 0;
//        }

        if (m_lastValue > 0) {
            if (desired < m_lastValue) {
                if (Math.abs(m_lastValue - desired) > m_rampRate) {
                    output = m_lastValue - m_rampRate;
                }
            }
        } else if (m_lastValue < 0) {
            if (desired > m_lastValue) {
                if (Math.abs(m_lastValue - desired) > m_rampRate) {
                    output = m_lastValue + m_rampRate;
                }
            }
        }

        m_lastValue = output;
        super.set(output);
    }
}
