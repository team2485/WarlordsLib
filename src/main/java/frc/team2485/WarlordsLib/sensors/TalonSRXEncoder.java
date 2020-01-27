package frc.team2485.WarlordsLib.sensors;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Sendable;

/**
 * A wrapper for interfacing with an encoder plugged into a TalonSRX.
 */
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
                return this.getQuadraturePosition() * m_distancePerRevolution / m_pulsesPerRevolution;
            case ANALOG:
                return this.getAnalogInRaw() * m_distancePerRevolution / m_pulsesPerRevolution;
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
                this.setPulseWidthPosition((int)(position * m_pulsesPerRevolution / m_distancePerRevolution), 0);
                break;
            case QUADRATURE:
                this.setQuadraturePosition((int)(position * m_pulsesPerRevolution / m_distancePerRevolution),0);
                break;
            case ANALOG:
                this.setAnalogPosition((int)(position * m_pulsesPerRevolution / m_distancePerRevolution), 0);
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
                return (this.getPulseWidthVelocity() * m_distancePerRevolution / m_pulsesPerRevolution)*10;
            case QUADRATURE:
                return (this.getQuadratureVelocity() * m_distancePerRevolution / m_pulsesPerRevolution)*10;
            case ANALOG:
                return (this.getAnalogInVel() * m_distancePerRevolution / m_pulsesPerRevolution)*10;
            default:
                return 0;
        }
    }

    private void reportError(ErrorCode code) {
        if (code != ErrorCode.OK) {
            DriverStation.reportWarning("TalonSRX Encoder Wrapper Error: " + code.toString(), true);
        }
    }
}
