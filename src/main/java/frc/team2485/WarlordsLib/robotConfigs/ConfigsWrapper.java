package frc.team2485.WarlordsLib.robotConfigs;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;

import java.util.HashMap;
import java.util.Map;
import java.util.function.*;

public class ConfigsWrapper implements LoadableConfigs, SavableConfigs {

    private RobotConfigs _robotConfigs;

    private String _category;

    // This is the constructor
    public ConfigsWrapper(String category, RobotConfigs robotConfigsInstance) {
        this._category = category;
        this._robotConfigs = robotConfigsInstance;
    }

    // Methods for getting config values by type
    @Override
    public String getString(String key, String backup) {
        return _robotConfigs.getString(_category, key, backup);
    }

    @Override
    public double getDouble(String key, double backup) {
        return _robotConfigs.getDouble(_category, key, backup);
    }

    @Override
    public int getInt(String key, int backup) {
        return _robotConfigs.getInt(_category, key, backup);
    }

    @Override
    public float getFloat(String key, float backup) { return _robotConfigs.getFloat(_category, key, backup); }

    @Override
    public long getLong(String key, long backup) { return _robotConfigs.getLong(_category, key, backup); }

    @Override
    public boolean getBoolean(String key, boolean backup) { return _robotConfigs.getBoolean(_category, key, backup); }

    // Methods for setting config values by type
    @Override
    public void put(String key, String value) { _robotConfigs.put(_category, key, value); }

    @Override
    public void put(String key, double value) { _robotConfigs.put(_category, key, value); }

    @Override
    public void put(String key, float value) { _robotConfigs.put(_category, key, value); }

    @Override
    public void put(String key, int value) { _robotConfigs.put(_category, key, value); }

    @Override
    public void put(String key, long value) { _robotConfigs.put(_category, key, value); }

    @Override
    public void put(String key, boolean value) { _robotConfigs.put(_category, key, value); }
}
