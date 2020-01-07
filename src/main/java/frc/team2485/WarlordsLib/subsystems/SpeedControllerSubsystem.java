package frc.team2485.WarlordsLib.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * General class with SpeedControllers for quick and dirty prototyping.
 */
public class SpeedControllerSubsystem extends SubsystemBase {

    private SpeedControllerGroup _speedControllers;

    public SpeedControllerSubsystem(SpeedController speedController, SpeedController... speedControllers) {
        _speedControllers = new SpeedControllerGroup(speedController, speedControllers);

        addChild(_speedControllers);
    }

    public void set(double speed) {
        _speedControllers.set(speed);
    }

    public void stop() {
        _speedControllers.stopMotor();
    }

}
