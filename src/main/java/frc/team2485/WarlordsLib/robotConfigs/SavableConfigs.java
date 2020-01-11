package frc.team2485.WarlordsLib.robotConfigs;

public interface SavableConfigs {

    void put(String key, String value);

    void put(String key, double value);

    void put(String key, float value);

    void put(String key, int value);

    void put(String key, long value);

    void put(String key, boolean value);
}
