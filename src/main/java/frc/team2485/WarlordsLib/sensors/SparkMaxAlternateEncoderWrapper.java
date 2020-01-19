package frc.team2485.WarlordsLib.sensors;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DriverStation;

public class SparkMaxAlternateEncoderWrapper implements Encoder {

    private double distancePerRevolution = 1;

    private double pulsesPerRevolution;

    private CANEncoder encoder;

    public SparkMaxAlternateEncoderWrapper(CANSparkMax spark, int pulsesPerRevolution) {
        this.pulsesPerRevolution = pulsesPerRevolution;

        this.encoder = spark.getAlternateEncoder(AlternateEncoderType.kQuadrature, pulsesPerRevolution);
    }

    public SparkMaxAlternateEncoderWrapper(int deviceId, int pulsesPerRevolution) {
        this(new CANSparkMax(deviceId, CANSparkMaxLowLevel.MotorType.kBrushless), pulsesPerRevolution);
    }

    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public void setPosition(double position) {
        handleCANError(encoder.setPosition(position));
    }

    @Override
    public void setDistancePerRevolution(double distance) {
        handleCANError(encoder.setPositionConversionFactor(distance));
        handleCANError(encoder.setVelocityConversionFactor(distance));
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity() * 60; // from distance per minute to distance per second
    }

    private void handleCANError(CANError error) {
        if (error != CANError.kOk) {
            DriverStation.reportWarning(error.toString(), true);
        }
    }

    public CANEncoder getEncoder() {
        return encoder;
    }
}
