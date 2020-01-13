package frc.team2485.WarlordsLib.sensors;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.SparkMax;

public class SparkMaxEncoderWrapper implements Encoder {

    public enum SparkMaxEncoderType {
        HALL_SENSOR, QUADRATURE
    }

    private double distancePerRevolution = 1;

    private double ticksPerRevolution;

    private CANSparkMax spark;

    private CANEncoder sensor;

    private SparkMaxEncoderType encoderType;

    public SparkMaxEncoderWrapper(CANSparkMax spark, SparkMaxEncoderType encoderType, int ticksPerRevolution) {
        this.spark = spark;
        this.ticksPerRevolution = ticksPerRevolution;
        this.encoderType = encoderType;

        switch (encoderType) {
            case QUADRATURE:
                this.sensor = spark.getEncoder(EncoderType.kQuadrature, ticksPerRevolution);
                break;
            case HALL_SENSOR:
                this.sensor = spark.getEncoder();
                break;
        }
    }

    @Override
    public double getPosition() {
        return sensor.getPosition();
    }

    @Override
    public void setPosition(double position) {
        sensor.setPosition(position);
    }

    @Override
    public void setDistancePerRevolution(double distance) {
        sensor.setPositionConversionFactor(distance);
        sensor.setVelocityConversionFactor(distance);
    }

    @Override
    public double getVelocity() {
        return sensor.getVelocity() * 60; // from distance per minute to distance per second
    }
}
