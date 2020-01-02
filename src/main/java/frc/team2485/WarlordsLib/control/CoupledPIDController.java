package frc.team2485.WarlordsLib.control;

import edu.wpi.first.wpilibj.controller.PIDController;

public class CoupledPIDController extends PIDController {

    /**
     * Allocates a PIDController with the given constants for Kp, Ki, and Kd and a default period of
     * 0.02 seconds.
     *
     * @param Kp The proportional coefficient.
     * @param Ki The integral coefficient.
     * @param Kd The derivative coefficient.
     */
    public CoupledPIDController(double Kp, double Ki, double Kd) {
        super(Kp, Ki, Kd);
    }

    /**
     * Allocates a PIDController with the given constants for Kp, Ki, and Kd.
     *
     * @param Kp     The proportional coefficient.
     * @param Ki     The integral coefficient.
     * @param Kd     The derivative coefficient.
     * @param period The period between controller updates in seconds.
     */
    public CoupledPIDController(double Kp, double Ki, double Kd, double period) {
        super(Kp, Ki, Kd, period);
    }


}
