package frc.team2485.WarlordsLib.motorcontrol;

import com.revrobotics.CANSparkMax;

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
        this.clearFaults();
    }

    public void setFollowers(CANSparkMax slave, CANSparkMax... slaves) {
        slave.follow(this);
        for (CANSparkMax m : slaves) {
            m.follow(this);
        }
    }
}
