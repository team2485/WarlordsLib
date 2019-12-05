package frc.team2485.WarlordsLib.motorcontrol.vendor;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class WPI_SparkMax extends CANSparkMax implements SpeedController, Sendable {

    private String _name = "";
    private String _subsystem = "Ungrouped";


    /**
     * Create a new SPARK MAX Controller
     *
     * @param deviceID The device ID.
     * @param type     The motor type connected to the controller. Brushless motors
     *                 must be connected to their matching color and the hall sensor
     *                 plugged in. Brushed motors must be connected to the Red and
     */
    public WPI_SparkMax(int deviceID, MotorType type) {
        super(deviceID, type);

        LiveWindow.add(this);
        setName("Spark Max " + deviceID);
    }

    /**
     * Add a child component.
     *
     * @param child
     *            child component
     */
    protected final void addChild(Object child) {
        LiveWindow.addChild(this, child);
    }

    @Override
    public String getName() {
        return this._name;
    }

    @Override
    public void setName(String name) {
        this._name = name;
    }

    @Override
    public String getSubsystem() {
        return this._subsystem;
    }

    @Override
    public void setSubsystem(String subsystem) {
        this._subsystem = subsystem;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Speed Controller");
        builder.setSafeState(this::stopMotor);
        builder.addDoubleProperty("Value", this::get, this::set);
    }
}
