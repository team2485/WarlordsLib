package frc.team2485.WarlordsLib.control;

/**
 * A generic interface for classes with an input and output.
 * @author Nathan Sariowan
 */
public interface Processor {

    public double calculate(double input);

    public void disable();
}
