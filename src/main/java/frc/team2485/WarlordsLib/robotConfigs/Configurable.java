package frc.team2485.WarlordsLib.robotConfigs;

/**
 * Base interfaces for objects that can be loaded and saved with RobotConfigs.
 */
public interface Configurable {

    /**
     * Initializes this Configurable object.
     * @param builder Configurable builder
     */
    void initConfigurable(ConfigurableBuilder builder);
}
