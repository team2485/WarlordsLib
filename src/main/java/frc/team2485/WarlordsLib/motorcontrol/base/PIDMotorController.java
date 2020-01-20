package frc.team2485.WarlordsLib.motorcontrol.base;

public interface PIDMotorController {

    void setP(double p);

    double getP();

    void setI(double i);

    double getI();

    void setD(double d);

    double getD();

    void setSetpoint(double setpoint);

    double getSetpoint();

    void runPID();

    double getOutput();

    void reset();

    void setEncoderPosition(double position);
}
