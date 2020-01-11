package frc.team2485.WarlordsLib.robotConfigs;

/**
 * Interfaces for getting constant values by type
 */
public interface LoadableConfigs {

    String getString(String key, String backup);

    double getDouble(String key, double backup);

    int getInt(String key, int backup);

    float getFloat(String key, float backup);

    long getLong(String key, long backup);

    boolean getBoolean(String key, boolean backup) ;
}
