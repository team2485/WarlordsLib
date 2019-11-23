package frc.team2485.WarlordsLib.robotConfigs;

import edu.wpi.first.wpilibj.DriverStation;

import java.io.*;

/**
 * Singleton for saving robot configs/constants locally on roborio. Replaces ConstantsIO.
 *
 * @author Nathan Sariowan
 */
public class RobotConfigurator {

    private static volatile RobotConfigurator _instance;

    private RobotConfigsMap _configs;

    private static final String DEFAULT_SEPARATOR = ",";

    /**
     * Instantiates RobotConfigurator singleton. Does NOT load configs from file; please use {@link #loadConfigsFromFile(String)}
     *
     * @return Singleton instance of RobotConfigurator
     */
    public static RobotConfigurator getInstance() {
        if (_instance == null) {
            synchronized (RobotConfigurator.class) {
                if (_instance == null) {
                    _instance = new RobotConfigurator();
                }
            }
        }
        return _instance;
    }

    private RobotConfigurator() {
        _configs = new RobotConfigsMap();
    }

    public void loadConfigsFromFile(String filepath) {
        loadConfigsFromFile(filepath, DEFAULT_SEPARATOR);
    }

    public void loadConfigsFromFile(String filepath, String separator) {
        _configs.clear();
        File file = new File(filepath);
        try {
            if (file.createNewFile()) {
                DriverStation.reportWarning("Constants file not found! Creating new file at " + filepath, false);
            };
        } catch (IOException e) {
            e.printStackTrace();
        }

        try (BufferedReader reader = new BufferedReader(new FileReader(filepath))) {
            String row;
            while ((row = reader.readLine()) != null) {
                String[] data = row.split(separator);
                if (data.length == 3) {
                    _configs.put(data[0].trim(), data[1].trim(), data[2]);
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void saveConfigsToFile(String filepath) {
        saveConfigsToFile(filepath, DEFAULT_SEPARATOR);
    }

    public void saveConfigsToFile(String filepath, String separator) {
        try (FileWriter writer = new FileWriter(filepath)) {
            for (String category : _configs.keySet()) {
                for (String key : _configs.keySet(category)) {
                    if (_configs.containsKey(category, key)) {
                        writer.append(category);
                        writer.append(separator);
                        writer.append(key);
                        writer.append(separator);
                        writer.append(_configs.get(category, key));
                        writer.append("\n");
                    }
                }
            }

            writer.flush();
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public String getString(String category, String key, String backup) {
        return _configs.getStringOrBackup(category, key, backup);
    }

    public double getDouble(String category, String key, double backup) {
        return _configs.getDoubleOrBackup(category, key, backup);
    }

    public int getInt(String category, String key, int backup) {
        return _configs.getIntOrBackup(category, key, backup);
    }

    public float getFloat(String category, String key, float backup) {
        return _configs.getFloatOrBackup(category, key, backup);
    }

    public long getLong(String category, String key, long backup) {
        return _configs.getLongOrBackup(category, key, backup);
    }

    public boolean getBoolean(String category, String key, boolean backup) {
        return _configs.getBooleanOrBackup(category, key, backup);
    }

    public void put(String category, String key, String value) {
        _configs.put(category, key, value);
    }

    public void put(String category, String key, double value) {
        _configs.put(category, key, Double.toString(value));
    }

    public void put(String category, String key, float value) {
        _configs.put(category, key, Float.toString(value));
    }

    public void put(String category, String key, int value) {
        _configs.put(category, key, Integer.toString(value));
    }

    public void put(String category, String key, long value) {
        _configs.put(category, key, Long.toString(value));
    }

    public void put(String category, String key, boolean value) {
        _configs.put(category, key, Boolean.toString(value));
    }

}
