package frc.team2485.WarlordsLib.robotConfigs;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

import java.util.function.*;

public interface ConfigurableBuilder {

    /**
     * Add a double property.
     *
     * @param key     property name
     * @param getter  getter function (returns current value)
     * @param setter  setter function (sets new value)
     */
    void addDoubleProperty(String key, DoubleSupplier getter, DoubleConsumer setter);

    /**
     * Add a integer property.
     *
     * @param key     property name
     * @param getter  getter function (returns current value)
     * @param setter  setter function (sets new value)
     */
    void addIntProperty(String key, IntSupplier getter, IntConsumer setter);

    /**
     * Add a float property.
     *
     * @param key     property name
     * @param getter  getter function (returns current value)
     * @param setter  setter function (sets new value)
     */
    void addFloatProperty(String key, Supplier<Float> getter, Consumer<Float> setter);


    /**
     * Add a long property.
     *
     * @param key     property name
     * @param getter  getter function (returns current value)
     * @param setter  setter function (sets new value)
     */
    void addLongProperty(String key, Supplier<Long> getter, Consumer<Long> setter);

    /**
     * Add a string property.
     *
     * @param key     property name
     * @param getter  getter function (returns current value)
     * @param setter  setter function (sets new value)
     */
    void addStringProperty(String key, Supplier<String> getter, Consumer<String> setter);

    /**
     * Add a boolean property.
     *
     * @param key     property name
     * @param getter  getter function (returns current value)
     * @param setter  setter function (sets new value)
     */
    void addBooleanProperty(String key, Supplier<Boolean> getter, Consumer<Boolean> setter);

}
