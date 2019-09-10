package frc.team2485.WarlordsLib.control;

/**
 * A generic interface for control systems with a single setpoint and input
 * @author Nathan Sariowan
 */
public interface Controller extends Processor {

    public void setSetpoint(double setpoint);

}
