package frc.team2485.WarlordsLib;

public interface PositionPIDSubsystem extends Tunable {

  double getEncoderPosition();

  void runPositionPID(double position);

  boolean atPositionSetpoint();
}
