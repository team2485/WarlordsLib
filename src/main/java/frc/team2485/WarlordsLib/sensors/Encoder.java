package frc.team2485.WarlordsLib.sensors;

public interface Encoder {

    double getPosition();

    void setPosition(double position);

    void setDistancePerRevolution(double distance);

    double getVelocity();
}
