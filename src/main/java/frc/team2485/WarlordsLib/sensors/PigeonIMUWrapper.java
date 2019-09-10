package frc.team2485.WarlordsLib.sensors;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

public class PigeonIMUWrapper extends PigeonIMU {

    public static enum Units {
        DEGS, RADS
    }

    private Units units;

    public PigeonIMUWrapper(int deviceNumber, Units unit) {
        super(deviceNumber);
        this.units = unit;
    }

    public PigeonIMUWrapper(TalonSRX talonSrx, Units units) {
        super(talonSrx);
        this.units = units;
    }

    public void reset() {
        this.setFusedHeading(0, 50);
        this.setYaw(0, 50);
    }

    public double getPosition() {
        return (units == Units.RADS) ? Math.PI / 180 * -1 * this.getFusedHeading() : -1 * this.getFusedHeading();
    }

    public double getVelocity() {
        double[] xyz = new double[3];
        this.getRawGyro(xyz);

        return (units == Units.RADS) ? Math.PI / 180 * -1 * xyz[2] : -1 * xyz[2];
    }
}
