package frc.team2485.WarlordsLib.sensors;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class WL_PigeonIMU extends PigeonIMU implements Sendable, Sensor {

    private String _name = "";
    private String _subsystem = "Ungrouped";

    public static enum Units {
        DEGS, RADS
    }

    private Units _units;

    public WL_PigeonIMU(int deviceNumber, Units units) {
        super(deviceNumber);
        this._units = units;

        LiveWindow.add(this);
        setName("Spark Max " + deviceNumber);
    }

    public WL_PigeonIMU(TalonSRX talonSrx, Units units) {
        super(talonSrx);
        this._units = units;

        LiveWindow.add(this);
        setName("Spark Max " + talonSrx.getDeviceID());
    }

    public void reset() {
        this.setFusedHeading(0, 50);
        this.setYaw(0, 50);
    }

    public double getPosition() {
        return (_units == Units.RADS) ? Math.PI / 180 * -1 * this.getFusedHeading() : -1 * this.getFusedHeading();
    }

    public double getVelocity() {
        double[] xyz = new double[3];
        this.getRawGyro(xyz);

        return (_units == Units.RADS) ? Math.PI / 180 * -1 * xyz[2] : -1 * xyz[2];
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

        builder.addDoubleProperty("Angular Position", this::getPosition, null);
        builder.addDoubleProperty("Angular Velocity", this::getVelocity, null);
    }

}
