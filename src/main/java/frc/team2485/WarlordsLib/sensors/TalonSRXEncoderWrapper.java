package frc.team2485.WarlordsLib.sensors;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Copied from previous year; @todo look at available SensorCollection methods
 */
public class TalonSRXEncoderWrapper {

	private static final int DEFAULT_TICKS_PER_REVOLUTION = 4096;

	private double distancePerRevolution;
	private double offset = 0;

	private double ticksPerRevolution;

	private TalonSRX talon;

	public TalonSRXEncoderWrapper(TalonSRX motorController) {
		this(motorController, DEFAULT_TICKS_PER_REVOLUTION);
	}

	public TalonSRXEncoderWrapper(TalonSRX motorController, int ticksPerRevolution) {
		this.talon = motorController;
		this.ticksPerRevolution = ticksPerRevolution;
	}

    public void setDistancePerRevolution(double distancePerRevolution) {
		this.distancePerRevolution = distancePerRevolution;
	}

	public double getDistancePerRevolution(double distancePerRevolution) {
		return this.distancePerRevolution;
	}

	public double getPosition() {
	    return ((double)(getQuadraturePosition() + offset)/ticksPerRevolution)*distancePerRevolution;
    }

    public double getVelocity() {
        return ((double)(talon.getSelectedSensorVelocity(0))/ticksPerRevolution)*10*distancePerRevolution;
    }

    public void setOffset(double position) {
	    this.offset = position * ticksPerRevolution / distancePerRevolution - getQuadraturePosition();
    }

    public SensorCollection getSensor() {
		return talon.getSensorCollection();
	}

    public double getQuadraturePosition() {
        return talon.getSensorCollection().getQuadraturePosition();
    }
}
