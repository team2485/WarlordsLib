package frc.team2485.WarlordsLib.robotConfigs;

/**
 * Base interfaces for objects that can be loaded and saved with RobotConfigs.
 */
public interface Configurable {

    /**
     * Loads configs with RobotConfigs
     * @param configs config to be loaded, type LoadableConfigs
     */
    void loadConfigs(LoadableConfigs configs);

    /**
     * Saves configs with RobotConfigs
     * @param configs config to be saved, type LoadableConfigs
     */
    void saveConfigs(SavableConfigs configs);

}
