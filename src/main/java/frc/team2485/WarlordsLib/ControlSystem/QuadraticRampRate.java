package common.control;

/** Quadratic:
 * @author Ian Lillie
 * @author Mark Rifkin
 */
public class QuadraticRampRate extends RampRate {

    protected double upRampRateA, upRampRateB, downRampRateA, downRampRateB;

    public QuadraticRampRate(){
        super();

        this.maxErrorToDesired = 0;
    }

    public void setRampRates(double upRampRateA, double upRampRateB, double downRampRateA, double downRampRateB) {
        this.upRampRateA = upRampRateA;
        this.upRampRateB = upRampRateB;
        this.downRampRateA = downRampRateA;
        this.downRampRateB = downRampRateB;

    }

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
            if (Math.abs(desired - this.lastValue) > this.upRampRateB + this.upRampRateA * this.scaledError) {
                if (desired > 0) {
                    this.lastValue += this.upRampRateB + this.upRampRateA * this.scaledError;
                } else {
                    this.lastValue -= this.upRampRateB + this.upRampRateA * this.scaledError;
                }
            } else {
                this.lastValue = desired;
            }
        } else {
            if (Math.abs(desired - this.lastValue) > this.downRampRateB + this.downRampRateA * this.scaledError) {
                if (lastValue > 0) {
                    this.lastValue -= this.downRampRateB + this.downRampRateA / this.scaledError;
                } else {
                    this.lastValue += this.downRampRateB + this.downRampRateA / this.scaledError;
                }
            } else {
                this.lastValue = desired;
            }

        }

        return this.lastValue;
    }
}