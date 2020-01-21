package frc.team2485.WarlordsLib;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Extension of DifferentialDrive that deals with some of the discrepancies of WPI's DifferentialDrive;
 *
 * No deadbands and input squaring by default (the idea is that all scaling happens in RobotContainer, not the subsystem,
 *      as that is a driver preference. Use {@link frc.team2485.WarlordsLib.oi.Deadband} to deadband/scale controllers).
 *
 * RIGHT SIDE IS INVERTED!
 *
 */
public class WL_DifferentialDrive extends DifferentialDrive {

    /**
     * Construct a DifferentialDrive.
     *
     * <p>To pass multiple motors per side, use a {@link edu.wpi.first.wpilibj.SpeedControllerGroup}. If a motor needs to be
     * inverted, do so before passing it in.
     *
     * @param leftMotor left motor
     * @param rightMotor right motor
     */
    public WL_DifferentialDrive(SpeedController leftMotor, SpeedController rightMotor) {
        super(leftMotor, rightMotor);
        this.setDeadband(0);
    }

    /**
     * Arcade drive method for differential drive platform.
     * The calculated values are NOT squared.
     *
     * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
     *                  positive.
     */
    @Override
    public void arcadeDrive(double xSpeed, double zRotation) {
        super.arcadeDrive(xSpeed, zRotation, false);
    }

    /**
     * Tank drive method for differential drive platform.
     * The calculated values are NOT squared.
     *
     * @param leftSpeed  The robot's left side speed along the X axis [-1.0..1.0]. Forward is
     *                   positive.
     * @param rightSpeed The robot's right side speed along the X axis [-1.0..1.0]. Forward is
     *                   positive.
     */
    @Override
    public void tankDrive(double leftSpeed, double rightSpeed) {
        super.tankDrive(leftSpeed, rightSpeed, false);
    }


}
