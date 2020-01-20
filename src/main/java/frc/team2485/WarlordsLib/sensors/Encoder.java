package frc.team2485.WarlordsLib.sensors;

public interface Encoder {

    /**
     * Get position of the encoder scaled by setDistancePerRevolution
     * @return position of encoder
     */
    double getPosition();

    /**
     * Reset position of encoder
     * @param position position to set encoder
     */
    void setPosition(double position);

    /**
     * Set distance per revolution scale factor for encoder
     * @param distance distance per rev for encoder
     */
    void setDistancePerRevolution(double distance);

    /**
     * Get velocity of encoder scaled by setDistancePerRevolution
     * @return velocity of encoder
     */
    double getVelocity();
}
