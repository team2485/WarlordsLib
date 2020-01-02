package frc.team2485.WarlordsLib.motorcontrol;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.revrobotics.CANSparkMax;
import frc.team2485.WarlordsLib.motorcontrol.base.WPI_SparkMax;

/**
 * Warlords wrapper for Spark Max with convenience functions.
 */
public class WL_SparkMax extends WPI_SparkMax {

    /**
     * Create a new SPARK MAX Controller
     *
     * @param deviceID The device ID.
     * @param type     The motor type connected to the controller. Brushless motors
     *                 must be connected to their matching color and the hall sensor
     */
    public WL_SparkMax(int deviceID, MotorType type) {
        super(deviceID, type);
        this.restoreFactoryDefaults();
    }

    public void setFollowers(WL_SparkMax slave, WL_SparkMax... slaves) {
        slave.follow(this);
        for (CANSparkMax m : slaves) {
            m.follow(this);
        }
    }
}
