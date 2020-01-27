package frc.team2485.WarlordsLib.sensors;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Sendable;

public class TalonSRXEncoder extends SensorCollection implements EncoderWrapper {

    public enum TalonSRXEncoderType {
        ABSOLUTE, QUADRATURE, ANALOG
    }

    private double m_distancePerRevolution = 1;

    private double m_pulsesPerRevolution;

    private TalonSRXEncoderType m_encoderType;

    public TalonSRXEncoder(TalonSRX motorController, TalonSRXEncoderType encoderType, int pulsesPerRevolution) {
        super(motorController);
        this.m_pulsesPerRevolution = pulsesPerRevolution;
        this.m_encoderType = encoderType;
    }

    public TalonSRXEncoder(int deviceId, TalonSRXEncoderType encoderType, int pulsesPerRevolution) {
        this(new TalonSRX(deviceId), encoderType, pulsesPerRevolution);
    }

    public void setEncoderType(TalonSRXEncoderType encoderType) {
        this.m_encoderType = encoderType;
    }

    public TalonSRXEncoderType getEncoderType() {
        return this.m_encoderType;
    }

    /**
     * Get position of encoder according to set distance per revolution
     * @return position
     */
    @Override
    public double getPosition() {
        switch (m_encoderType) {
            case ABSOLUTE:
                return this.getPulseWidthPosition() * m_distancePerRevolution / m_pulsesPerRevolution;
            case QUADRATURE:
                return convertTicksToDistance(this.getQuadraturePosition());
            case ANALOG:
                return convertTicksToDistance(this.getAnalogInRaw());
            default:
                return 0;
        }
    }

    /**
     * Reset encoder position
     * @param position position to set to
     */
    @Override
    public void resetPosition(double position) {
        switch (m_encoderType) {
            case ABSOLUTE:
                this.setPulseWidthPosition(convertDistanceToTicks(position),0);
                break;
            case QUADRATURE:
                this.setQuadraturePosition(convertDistanceToTicks(position),0);
                break;
            case ANALOG:
                this.setAnalogPosition(convertDistanceToTicks(position), 0);
                break;
        }
    }

    /**
     *  Set scaling factor for encoder
     * @param distance distance per revolution
     */
    @Override
    public void setDistancePerRevolution(double distance) {
        this.m_distancePerRevolution = distance;
    }

    /**
     * Get velocity in distance per second.
     *
     * @return velocity based on given distance per revolution and CPR.
     */
    @Override
    public double getVelocity() {
        switch (m_encoderType) {
            case ABSOLUTE:
                return convertTicksToDistance(this.getPulseWidthVelocity())*10;
            case QUADRATURE:
                return convertTicksToDistance(this.getQuadratureVelocity())*10;
            case ANALOG:
                return convertTicksToDistance(this.getAnalogInVel())*10;
            default:
                return 0;
        }
    }

    private int convertDistanceToTicks(double distance) {
        return (int) (distance * m_pulsesPerRevolution / m_distancePerRevolution);
    }

    private double convertTicksToDistance(int ticks) {
        return ticks * m_distancePerRevolution / m_pulsesPerRevolution;
    }
}
