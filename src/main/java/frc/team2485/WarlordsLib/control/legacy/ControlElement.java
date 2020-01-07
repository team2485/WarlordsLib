package frc.team2485.WarlordsLib.control.legacy;

/**
 * A generic interface for classes with an input and output.
 */
public interface ControlElement {

    public double calculate(double input);

    public void disable();
}
