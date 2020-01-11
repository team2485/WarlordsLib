package frc.team2485.WarlordsLib.robotConfigs;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

import java.io.*;
import java.util.Map;
import java.util.WeakHashMap;

/**
 * Singleton for saving robot configs/constants locally on roborio. Replaces ConstantsIO.
 *
 * Call me by using RobotConfigs.getInstance()
 */
public class RobotConfigs {

    // Volatile is alternative to synchronize for allowing multiple threads to access methods without interfering with variable data
    // Volatile does not cache values
    private static volatile RobotConfigs _instance;

    private RobotConfigsMap _configs;

    private ConfigurableRegistry _configurableRegistry;

    private static final String DEFAULT_CSV_SEPARATOR = ",";

    private boolean _loaded;

    /**
     * Instantiates RobotConfigs singleton. Does NOT load configs from file; please use {@link #loadConfigsFromFile(String)}
     *
     * @return Singleton instance of RobotConfigs
     */
    public static RobotConfigs getInstance() {
        if (_instance == null) {
            synchronized (RobotConfigs.class) {
                if (_instance == null) {
                    _instance = new RobotConfigs();
                }
            }
        }
        return _instance;
    }

    private RobotConfigs() {
        _configs = new RobotConfigsMap();
        _configurableRegistry = new ConfigurableRegistry();
        _loaded = false;
    }

    private boolean configsLoadedFromFile() {
        return this._loaded;
    }

    private void setConfigsLoadedFromFile(boolean loaded) {
        this._loaded = loaded;
    }

    /**
     * It is recommended to run this BEFORE running RobotContainer so it may access loaded values!
     * @param filepath the location of the file
     */
    public void loadConfigsFromFile(String filepath) {
        loadConfigsFromFile(filepath, DEFAULT_CSV_SEPARATOR);
    }

    /**
     * Loads configs from file with parsing
     * Use when file is not default CSV
     * @param filepath location of file
     * @param separator delimiter used in file
     */
    public void loadConfigsFromFile(String filepath, String separator) {
        _configs.clear();
        File file = new File(filepath);
        try {
            if (file.createNewFile()) {
                DriverStation.reportWarning("Constants file not found! Creating new file at " + filepath, false);
            }
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
            setConfigsLoadedFromFile(true);
        } catch (IOException e) {
            e.printStackTrace();
        }

        _configurableRegistry.updateAll();
    }

    /**
     * Saves config to file
     * @param filepath location to save config file
     */
    public void saveConfigsToFile(String filepath) {
        saveConfigsToFile(filepath, DEFAULT_CSV_SEPARATOR);
    }

    /**
     * Saves config to file but with parsing
     * Use when special delimiter wanted in file (not default CSV)
     * @param filepath location to save config file
     * @param separator desired delimiter for data
     */
    public void saveConfigsToFile(String filepath, String separator) {

        _configurableRegistry.saveAll();

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

    // returns config info by type
    public String getString(String category, String key, String backup) {
        checkConfigsLoaded();
        return _configs.getStringOrBackup(category, key, backup);
    }

    public double getDouble(String category, String key, double backup) {
        checkConfigsLoaded();
        return _configs.getDoubleOrBackup(category, key, backup);
    }

    public int getInt(String category, String key, int backup) {
        checkConfigsLoaded();
        return _configs.getIntOrBackup(category, key, backup);
    }

    public float getFloat(String category, String key, float backup) {
        checkConfigsLoaded();
        return _configs.getFloatOrBackup(category, key, backup);
    }

    public long getLong(String category, String key, long backup) {
        checkConfigsLoaded();
        return _configs.getLongOrBackup(category, key, backup);
    }

    public boolean getBoolean(String category, String key, boolean backup) {
        checkConfigsLoaded();
        return _configs.getBooleanOrBackup(category, key, backup);
    }

    private void checkConfigsLoaded() {
        if (!configsLoadedFromFile()) {
            DriverStation.reportWarning("RobotConfigs has not loaded a file yet, so no constants have been loaded. Make sure to run method loadConfigsFromFile!", true);
        }
    }

    // sets config info by type
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

    /**
     * saves configurable (probably should just read method name)
     */
    public void saveConfigurable(String category, Configurable configurable) {
        configurable.saveConfigs(new ConfigsWrapper(category, this));
    }

    /**
     * loads configurable
     */
    public void loadConfigurable(String category, Configurable configurable) {
        configurable.loadConfigs(new ConfigsWrapper(category, this));
    }

    public void addConfigurable(String category, Configurable configurable) {
        _configurableRegistry.addConfigurable(category, configurable);
    }

    /**
     * Registry of all Configurables.
     */
    private class ConfigurableRegistry {

        private WeakHashMap<String, Configurable> _configurables;

        private ConfigurableRegistry() {
            _configurables = new WeakHashMap<>();
        }

        private void addConfigurable(String category, Configurable configurable) {
            configurable.loadConfigs(new ConfigsWrapper(category, RobotConfigs.getInstance()));
            _configurables.put(category, configurable);
        }

        private void updateAll() {
            System.out.println("Updating");
            for (Map.Entry<String, Configurable> entry : _configurables.entrySet()) {
                entry.getValue().loadConfigs(new ConfigsWrapper(entry.getKey(), RobotConfigs.getInstance()));
            }
        }

        private void saveAll() {
            for (Map.Entry<String, Configurable> entry : _configurables.entrySet()) {
                entry.getValue().saveConfigs(new ConfigsWrapper(entry.getKey(), RobotConfigs.getInstance()));
            }
        }
    }

}
