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
	private ControlMode m_controlMode;

	/**
	 * Constructor
	 *
	 * @param deviceNumber
	 */
	public TalonSRXWrapper(ControlMode controlMode, int deviceNumber) {
		super(deviceNumber);
		this.m_controlMode = controlMode;
	}

	public TalonSRXWrapper(int deviceNumber) {
		this(ControlMode.PercentOutput, deviceNumber);
	}

	@Override
	public void set(double speed) {
		super.set(m_controlMode, speed);
	}

	public ControlMode getControlMode() {
		return m_controlMode;
	}

	public void setControlMode(ControlMode controlMode) {
		this.m_controlMode = controlMode;
	}
}
