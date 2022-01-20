package frc.team2485.WarlordsLib;

public interface VelocityPIDSubsystem extends Tunable {

  double getEncoderVelocity();

  void runVelocityPID(double velocity);

  boolean atVelocitySetpoint();
}
