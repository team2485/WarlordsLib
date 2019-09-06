package frc.team2485.WarlordsLib.control;

public interface ControlSystem {

    public void setSetpoint(double setpoint);

    public double calculate(double measurement);


}
