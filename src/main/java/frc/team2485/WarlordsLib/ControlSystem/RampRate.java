package common.control;

import edu.wpi.first.wpilibj.PIDOutput;

/**
 * Generic class to ramp an output (velocity, voltage, current, etc). Has unique up and down rates. 
 * @author Jeremy McCulloch
 */
public class RampRate extends WarlordsControlSystem {
	
	protected double lastValue, upRampRate, downRampRate;

	protected double maxErrorToDesired;
	protected double scaledError;
	
	public RampRate() { 
		this.lastValue = 0;
	}

	public void setRampRates(double upRampRate, double downRampRate) {
		this.upRampRate = upRampRate;
		this.downRampRate = downRampRate;
	}

	/**
	 * Used to get next value by moving from the last value toward the desired value, without exceeding the ramp rate
	 * @param desired target value
	 * @return actual value
	 */
	
	public double getNextValue(double desired) {
		if ((this.lastValue > 0 && desired < 0) || (this.lastValue < 0 && desired > 0)) {
			desired = 0; // makes sure desired and lastValue have the same sign to make math easy
		}
		//System.out.println("Desired: " + desired);
		double errorToDesired = Math.abs(desired - this.lastValue);


		if (errorToDesired > this.maxErrorToDesired) {
			this.maxErrorToDesired = errorToDesired;
		}

		this.scaledError = errorToDesired / this.maxErrorToDesired; //scaled error is now mapped between 0 and 1

		if (Math.abs(desired) > Math.abs(this.lastValue)) {
			if (Math.abs(desired - this.lastValue) > upRampRate) {
				if (desired > 0) {
					this.lastValue += this.upRampRate;
				} else {
					this.lastValue -= this.upRampRate;
				}
			} else {
				this.lastValue = desired;
			}
		} else {
			if (Math.abs(desired - this.lastValue) > this.downRampRate) {
				if (this.lastValue > 0) {
					this.lastValue -= this.downRampRate;
				} else {
					this.lastValue += this.downRampRate;
				}
			} else {
				this.lastValue = desired;
			}
		}

		return this.lastValue;

	}


	
	/**
	 * Used to immediately set the last value, potentially not following ramp rate
	 * @param lastValue new value to be treated as the last value
	 */
	public void setLastValue(double lastValue) {
		this.lastValue = lastValue;
	}

	@Override
	protected void calculate() {
		double val = getNextValue(setpoint);
		for (PIDOutput out : outputs) {
			out.pidWrite(val);
		}
	}
	
	@Override
	public void disable() {
		setLastValue(0);
		setpoint = 0;
		super.disable();
	}
}
