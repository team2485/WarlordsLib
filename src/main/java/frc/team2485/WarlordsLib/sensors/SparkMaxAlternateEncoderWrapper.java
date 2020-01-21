package frc.team2485.WarlordsLib.sensors;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DriverStation;

public class SparkMaxAlternateEncoderWrapper implements EncoderWrapper {

    private CANEncoder m_encoder;

    public SparkMaxAlternateEncoderWrapper(CANSparkMax spark, int pulsesPerRevolution) {
        this.m_encoder = spark.getAlternateEncoder(AlternateEncoderType.kQuadrature, pulsesPerRevolution);
    }

    public SparkMaxAlternateEncoderWrapper(int deviceId, int pulsesPerRevolution) {
        this(new CANSparkMax(deviceId, CANSparkMaxLowLevel.MotorType.kBrushless), pulsesPerRevolution);
    }

    /**
     * Get position in rotations * distance per revolution
     * @return distance
     */
    @Override
    public double getPosition() {
        return m_encoder.getPosition();
    }

    /**
     * Reset position
     * @param position position to reset to in distance taking into account conversion factor.
     */
    @Override
    public void setPosition(double position) {
        handleCANError(m_encoder.setPosition(position));
    }

    /**
     * Set the distance per revolution of the encoder
     * @param distance distance per rev
     */
    @Override
    public void setDistancePerRevolution(double distance) {
        handleCANError(m_encoder.setPositionConversionFactor(distance));
        handleCANError(m_encoder.setVelocityConversionFactor(distance));
    }

    /**
     * Get velocity of the encoder is distance per second
     * @return distance per second
     */
    @Override
    public double getVelocity() {
        return m_encoder.getVelocity() / 60; // from distance per minute to distance per second
    }

    private void handleCANError(CANError error) {
        if (error != CANError.kOk) {
            DriverStation.reportWarning(error.toString(), true);
        }
    }

    /**
     * Sets this encoder to be inverted
     * @param inverted whether encoder is inverted
     */
    public void setInverted(boolean inverted) {
        m_encoder.setInverted(inverted);
    }

    /**
     * Get this encoder
     * @return the CANEncoder
     */
    public CANEncoder getEncoder() {
        return m_encoder;
    }
}
