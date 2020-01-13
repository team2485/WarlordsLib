package frc.team2485.WarlordsLib.sensors;

public interface Encoder {

    double getPosition();

    void setPosition(double position);

    void setDistancePerRevolution(double distance);

    double getVelocity();

    default double convertTicksToDistance(int ticks, double distancePerRevolution, double ticksPerRevolution) {
        return ticks * distancePerRevolution / ticksPerRevolution;
    }

    default int convertDistanceToTicks(double distance, double distancePerRevolution, double ticksPerRevolution) {
        return (int) (distance * ticksPerRevolution / distancePerRevolution);
    }
}
