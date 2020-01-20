package frc.team2485.WarlordsLib.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * General class with SpeedControllers for quick and dirty prototyping.
 */
public class SpeedControllerSubsystem extends SubsystemBase {

    private SpeedControllerGroup m_speedControllers;

    public SpeedControllerSubsystem(SpeedController speedController, SpeedController... speedControllers) {
        m_speedControllers = new SpeedControllerGroup(speedController, speedControllers);
    }

    public void set(double speed) {
        m_speedControllers.set(speed);
    }

    public double get() {
        return m_speedControllers.get();
    }

    public void stop() {
        m_speedControllers.stopMotor();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Speed Controllers", this::get, this::set);
    }
}
