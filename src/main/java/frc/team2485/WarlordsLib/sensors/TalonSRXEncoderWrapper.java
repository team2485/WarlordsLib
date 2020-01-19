package frc.team2485.WarlordsLib.sensors;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Copied from previous year; @todo look at available SensorCollection methods
 */
public class TalonSRXEncoderWrapper implements Encoder {


    public enum TalonSRXEncoderType {
        ABSOLUTE, QUADRATURE, ANALOG
    }

    private double distancePerRevolution = 1;

    private double pulsesPerRevolution;

    private SensorCollection sensor;

    private TalonSRXEncoderType encoderType;

    public TalonSRXEncoderWrapper(TalonSRX motorController, TalonSRXEncoderType encoderType, int pulsesPerRevolution) {
        this.pulsesPerRevolution = pulsesPerRevolution;
        this.encoderType = encoderType;
        this.sensor = motorController.getSensorCollection();
    }

    public TalonSRXEncoderWrapper(int deviceId, TalonSRXEncoderType encoderType, int pulsesPerRevolution) {
        this(new TalonSRX(deviceId), encoderType, pulsesPerRevolution);
    }

    @Override
    public double getPosition() {
        switch (encoderType) {
            case ABSOLUTE:
                return convertTicksToDistance(sensor.getPulseWidthPosition());
            case QUADRATURE:
                return convertTicksToDistance(sensor.getQuadraturePosition());
            case ANALOG:
                return convertTicksToDistance(sensor.getAnalogInRaw());
            default:
                return 0;
        }
    }

    @Override
    public void setPosition(double position) {
        switch (encoderType) {
            case ABSOLUTE:
                sensor.setPulseWidthPosition(convertDistanceToTicks(position),0);
                break;
            case QUADRATURE:
                sensor.setQuadraturePosition(convertDistanceToTicks(position),0);
                break;
            case ANALOG:
                sensor.setAnalogPosition(convertDistanceToTicks(position), 0);
                break;
        }
    }

    @Override
    public void setDistancePerRevolution(double distance) {
        this.distancePerRevolution = distance;
    }

    /**
     * Get velocity in distance per second.
     *
     * @return velocity based on given distance per revolution and CPR.
     */
    @Override
    public double getVelocity() {
        switch (encoderType) {
            case ABSOLUTE:
                return convertTicksToDistance(sensor.getPulseWidthVelocity())/10;
            case QUADRATURE:
                return convertTicksToDistance(sensor.getQuadratureVelocity())/10;
            case ANALOG:
                return convertTicksToDistance(sensor.getAnalogInVel())/10;
            default:
                return 0;
        }
    }

    private int convertDistanceToTicks(double distance) {
        return (int) (distance * pulsesPerRevolution / distancePerRevolution);
    }

    private double convertTicksToDistance(int ticks) {
        return ticks * distancePerRevolution / pulsesPerRevolution;
    }
}
