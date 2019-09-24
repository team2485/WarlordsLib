package frc.team2485.WarlordsLib.motorcontrol;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * TalonSRX Wrapper compatible with WPILib, allows for setting of ControlMode (WPI_TalonSRX doesn't do this)
 */
public class TalonSRXWrapper extends WPI_TalonSRX {

	// Control mode when setting Talon
	private ControlMode defaultControlMode;

	/**
	 * Constructor
	 *
	 * @param deviceNumber
	 */
	public TalonSRXWrapper(ControlMode defaultControlMode, int deviceNumber) {
		super(deviceNumber);
		this.defaultControlMode = defaultControlMode;
	}

	public TalonSRXWrapper(int deviceNumber) {
		this(ControlMode.PercentOutput, deviceNumber);
	}

	@Override
	public void set(double speed) {
		super.set(defaultControlMode, speed);
	}

	public ControlMode getDefaultControlMode() {
		return defaultControlMode;
	}

	public void setDefaultControlMode(ControlMode defaultControlMode) {
		this.defaultControlMode = defaultControlMode;
	}
}
