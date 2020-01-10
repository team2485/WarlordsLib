package frc.team2485.WarlordsLib.robotConfigs;

/**
 * Base interfaces for objects that can be loaded and saved with RobotConfigs.
 */
public interface Configurable {

    void loadConfigs(LoadableConfigs configs);

    void saveConfigs(SavableConfigs configs);

}
