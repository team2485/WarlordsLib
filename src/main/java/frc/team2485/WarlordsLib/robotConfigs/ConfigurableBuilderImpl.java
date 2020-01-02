package frc.team2485.WarlordsLib.robotConfigs;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;

import java.util.HashMap;
import java.util.Map;
import java.util.function.*;

public class ConfigurableBuilderImpl implements ConfigurableBuilder {

    private HashMap<String, Property> _properties;

    private interface Property {
        void update(String category, String key);
        void save(String category, String key);
    }

    public ConfigurableBuilderImpl() {
        _properties = new HashMap<String, Property>();
    }

    /**
     * Update all Configurables from RobotConfigs. Calls all setters and sets their values based on corresponding keys
     * found in RobotConfigs.
     * @param category category key for this Configurable.
     */
    public void updateAll(String category) {
        for (Map.Entry<String, Property> entry : _properties.entrySet()) {
            entry.getValue().update(category, entry.getKey());
        }
    }

    /**
     * Saves all constants in Configurables to RobotConfigs. Calls all getters, takes values and saves to RobotConfigs.
     * @param category category key for this Configurable.
     */
    public void saveAll(String category) {
        for (Map.Entry<String, Property> entry : _properties.entrySet()) {
            entry.getValue().save(category, entry.getKey());
        }
    }

    /**
     * Add a string property.
     *
     * @param key     property name
     * @param getter  getter function (returns current value)
     * @param setter  setter function (sets new value)
     */
    public void addStringProperty(String key, Supplier<String> getter, Consumer<String> setter) {
        Property p = new Property() {

            @Override
            public void update(String category, String key) {
                setter.accept(RobotConfigs.getInstance().getString(category, key, getter.get()));
            }

            @Override
            public void save(String category, String key) {
                RobotConfigs.getInstance().put(category, key, getter.get());
            }
        };

        _properties.put(key, p);
    }

    /**
     * Add a boolean property.
     *
     * @param key    property name
     * @param getter getter function (returns current value)
     * @param setter setter function (sets new value)
     */
    @Override
    public void addBooleanProperty(String key, Supplier<Boolean> getter, Consumer<Boolean> setter) {
        Property p = new Property() {

            @Override
            public void update(String category, String key) {
                setter.accept(RobotConfigs.getInstance().getBoolean(category, key, getter.get()));
            }

            @Override
            public void save(String category, String key) {
                RobotConfigs.getInstance().put(category, key, getter.get());
            }
        };

        _properties.put(key, p);
    }

    /**
     * Add a double property.
     *
     * @param key    property name
     * @param getter getter function (returns current value)
     * @param setter setter function (sets new value)
     */
    @Override
    public void addDoubleProperty(String key, DoubleSupplier getter, DoubleConsumer setter) {
        Property p = new Property() {

            @Override
            public void update(String category, String key) {
                setter.accept(RobotConfigs.getInstance().getDouble(category, key, getter.getAsDouble()));
            }

            @Override
            public void save(String category, String key) {
                RobotConfigs.getInstance().put(category, key, getter.getAsDouble());
            }
        };

        _properties.put(key, p);
    }

    /**
     * Add a integer property.
     *
     * @param key    property name
     * @param getter getter function (returns current value)
     * @param setter setter function (sets new value)
     */
    @Override
    public void addIntProperty(String key, IntSupplier getter, IntConsumer setter) {
        Property p = new Property() {

            @Override
            public void update(String category, String key) {
                setter.accept(RobotConfigs.getInstance().getInt(category, key, getter.getAsInt()));
            }

            @Override
            public void save(String category, String key) {
                RobotConfigs.getInstance().put(category, key, getter.getAsInt());
            }
        };

        _properties.put(key, p);
    }

    /**
     * Add a float property.
     *
     * @param key    property name
     * @param getter getter function (returns current value)
     * @param setter setter function (sets new value)
     */
    @Override
    public void addFloatProperty(String key, Supplier<Float> getter, Consumer<Float> setter) {
        Property p = new Property() {

            @Override
            public void update(String category, String key) {
                setter.accept(RobotConfigs.getInstance().getFloat(category, key, getter.get()));
            }

            @Override
            public void save(String category, String key) {
                RobotConfigs.getInstance().put(category, key, getter.get());
            }
        };

        _properties.put(key, p);
    }

    /**
     * Add a long property.
     *
     * @param key    property name
     * @param getter getter function (returns current value)
     * @param setter setter function (sets new value)
     */
    @Override
    public void addLongProperty(String key, Supplier<Long> getter, Consumer<Long> setter) {
        Property p = new Property() {

            @Override
            public void update(String category, String key) {
                setter.accept(RobotConfigs.getInstance().getLong(category, key, getter.get()));
            }

            @Override
            public void save(String category, String key) {
                RobotConfigs.getInstance().put(category, key, getter.get());
            }
        };

        _properties.put(key, p);
    }

}
