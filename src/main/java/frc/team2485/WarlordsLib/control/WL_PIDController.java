package frc.team2485.WarlordsLib.control;

/**
 * All this does (for now) is make WPILib's PIDController implement our ControlSystem interface. The ControlSystem methods are satisfied in WPILib's PIDController
 * Believe it or not, WL stands for Warlords.
 */
public class WL_PIDController extends edu.wpi.first.wpilibj.controller.PIDController implements ControlSystem {

    public WL_PIDController(double Kp, double Ki, double Kd) {
        super(Kp, Ki, Kd);
    }

    public WL_PIDController(double Kp, double Ki, double Kd, double period) {
        super(Kp, Ki, Kd, period);
    }
}
