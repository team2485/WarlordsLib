package frc.team2485.WarlordsLib.control;

/**
 * A generic interface for control systems with a single setpoint and input
 */
public interface Controller extends ControlElement {

    public void setSetpoint(double setpoint);

}
