package frc.team2485.WarlordsLib.control.legacy;

import frc.team2485.WarlordsLib.control.legacy.ControlElement;

/**
 * Low pass input/output filter
 */
public class LowPassFilter implements ControlElement {

    private double filterCoefficient;
    private double lastValue;

    public void LowPassFilter(double filterCoefficient) {
        this.filterCoefficient = filterCoefficient;
    };

    public void setFilterCoefficient(double filterCoefficient) {
        this.filterCoefficient = filterCoefficient;
    }

    public double getFilterCoefficient() {
        return this.filterCoefficient;
    }

    @Override
    public double calculate(double input) {
        double output = lastValue + filterCoefficient * (input - lastValue);
        lastValue = output;
        return output;
    }

    @Override
    public void disable() {
        lastValue = 0;
    }
}
