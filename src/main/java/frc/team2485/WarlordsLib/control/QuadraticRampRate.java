package frc.team2485.WarlordsLib.control;

/**
 * Quadratic ramp rate based on Jeremey's ramp rate.
 * @author Ian Lillie
 * @author Mark Rifkin
 */
public class QuadraticRampRate implements Processor {

    private double lastValue;
    private double upRampRateA, upRampRateB, downRampRateA, downRampRateB;

    private double maxErrorToDesired;
    private double scaledError;

    public QuadraticRampRate(double upRampRateA, double upRampRateB, double downRampRateA, double downRampRateB) {
        setRampRates(upRampRateA, upRampRateB, downRampRateA, downRampRateB);
        this.lastValue = 0;
        this.maxErrorToDesired = 0;
    }

    public void setRampRates(double upRampRateA, double upRampRateB, double downRampRateA, double downRampRateB) {
        this.upRampRateA = upRampRateA;
        this.upRampRateB = upRampRateB;
        this.downRampRateA = downRampRateA;
        this. downRampRateB = downRampRateB;
    }

    public double getScaledError() {
        return this.scaledError;
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
            if (Math.abs(desired - lastValue) > upRampRateB + upRampRateA * scaledError) {
                if (desired > 0) {
                    output = lastValue + upRampRateB + upRampRateA * scaledError;
                } else {
                    output = lastValue - upRampRateB + upRampRateA * scaledError;
                }
            }
        } else {
            if (Math.abs(desired - lastValue) > downRampRateB + downRampRateA * scaledError) {
                if (lastValue > 0) {
                    output = lastValue - downRampRateB + downRampRateA / scaledError;
                } else {
                    output = lastValue + downRampRateB + downRampRateA / scaledError;
                }
            }
        }

        return output;
    }

    @Override
    public void disable() {
        this.lastValue = 0;
    }
}
