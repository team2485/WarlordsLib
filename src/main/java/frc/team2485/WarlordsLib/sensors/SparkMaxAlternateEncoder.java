package frc.team2485.WarlordsLib.sensors;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Used to interface with an alternate quadrature encoder plugged into a SparkMax. Not to be used with the Hall Sensor.
 */
public class SparkMaxAlternateEncoder extends CANEncoder implements EncoderWrapper {

    public SparkMaxAlternateEncoder(CANSparkMax spark, int pulsesPerRevolution) {
        super(spark, AlternateEncoderType.kQuadrature, pulsesPerRevolution);
    }

    public SparkMaxAlternateEncoder(int deviceId, int pulsesPerRevolution) {
        this(new CANSparkMax(deviceId, CANSparkMaxLowLevel.MotorType.kBrushless), pulsesPerRevolution);
    }

    /**
     * Reset position
     * @param position position to reset to in distance taking into account conversion factor.
     */
    @Override
    public void resetPosition(double position) {
        handleCANError(this.setPosition(position));
    }

    /**
     * Set the distance per revolution of the encoder
     * @param distance distance per rev
     */
    @Override
    public void setDistancePerRevolution(double distance) {
        handleCANError(this.setPositionConversionFactor(distance));
        handleCANError(this.setVelocityConversionFactor(distance));
    }

    /**
     * Get velocity of the encoder is distance per second
     * @return distance per second
     */
    @Override
    public double getVelocity() {
        return super.getVelocity() / 60; // from distance per minute to distance per second
    }

    private void handleCANError(CANError error) {
        if (error != CANError.kOk) {
            DriverStation.reportWarning("Spark Max Encoder Error: " + error.toString(), true);
        }
    }
}
