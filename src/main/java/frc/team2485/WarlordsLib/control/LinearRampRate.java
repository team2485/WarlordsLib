package frc.team2485.WarlordsLib.control;

/**
 * Based on Jeremey McCulloch's original LinearRampRate class.
 * Generic class to ramp an output (velocity, voltage, current, etc). Has unique up and down rates.
 * @author Jeremey McCulloch
 */
public class LinearRampRate implements ControlElement {

    private double lastValue;

    private double upRampRate, downRampRate;

    private double maxErrorToDesired;
    private double scaledError;

    public LinearRampRate(double upRampRate, double downRampRate) {
        this.upRampRate = upRampRate;
        this.downRampRate = downRampRate;
        this.lastValue = 0;
        this.maxErrorToDesired = 0;
    }

    public void setRampRates(double upRampRate, double downRampRate) {
        this.upRampRate = upRampRate;
        this.downRampRate = downRampRate;
    }

    @Override
    public double calculate(double input) {
        if ((lastValue > 0 && input < 0) || (lastValue < 0 && input > 0)) {
            input = 0; // makes sure desired and lastValue have the same sign to make math easy
        }
        //System.out.println("Desired: " + desired);
        double errorToDesired = Math.abs(input - lastValue);

        if (errorToDesired > maxErrorToDesired) {
            maxErrorToDesired = errorToDesired;
        }

        scaledError = errorToDesired / maxErrorToDesired; //scaled error is now mapped between 0 and 1

        double output = getNextValue(input, lastValue);

        lastValue = output;

        return output;
    }

    protected double getNextValue(double desired, double lastValue) {

        double output = desired;

        if (Math.abs(desired) > Math.abs(lastValue)) {
            if (Math.abs(desired - lastValue) > upRampRate) {
                if (desired > 0) {
                    output = lastValue + upRampRate;
                } else {
                    output = lastValue - upRampRate;
                }
            } else {
                output = desired;
            }
        } else {
            if (Math.abs(desired - lastValue) > downRampRate) {
                if (lastValue > 0) {
                    output = lastValue - downRampRate;
                } else {
                    output = lastValue + downRampRate;
                }
            } else {
                output = desired;
            }
        }

        return output;
    }

    /**
     * Used to immediately set the last value, potentially not following ramp rate
     * @param lastValue new value to be treated as the last value
     */
    public void setLastValue(double lastValue) {
        this.lastValue = lastValue;
    }

    public double getScaledError() {
        return scaledError;
    }

    @Override
    public void disable() {
        this.lastValue = 0;
    }

}
