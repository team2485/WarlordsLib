package frc.team2485.WarlordsLib.control;

import edu.wpi.first.wpilibj.controller.PIDController;

/**
 * All this does (for now) is make WPILib's PIDController implement our Controller interface. The Controller methods are satisfied in WPILib's PIDController
 * Believe it or not, WL stands for Warlords.
 */
public class WL_PIDController extends PIDController implements Controller {

    public WL_PIDController(double Kp, double Ki, double Kd) {
        super(Kp, Ki, Kd);
    }

    public WL_PIDController(double Kp, double Ki, double Kd, double period) {
        super(Kp, Ki, Kd, period);
    }

    @Override
    public void disable() {
        super.reset();
    }
}
