package frc.team2485.WarlordsLib.control;

public class FrictionController implements Controller {

    private double m_Kv;
    private double m_Vmax;

    private double m_setpoint;

    public FrictionController(double kV, double vMax) {
        this.m_Kv = kV;
        this.m_Vmax = vMax;
    }

    public double getSetpoint() {
        return m_setpoint;
    }

    @Override
    public void setSetpoint(double setpoint) {
        this.m_setpoint = setpoint;
    }

    public double getV() {
        return m_Kv;
    }

    public void setV(double m_kV) {
        this.m_Kv = m_kV;
    }

    public double getVmax() {
        return m_Vmax;
    }

    public void setVmax(double m_vMax) {
        this.m_Vmax = m_vMax;
    }

    @Override
    public double calculate(double input) {
        double output;
        if(m_setpoint >= 0.25){
            output = m_Kv * ((m_Vmax - input) / m_Vmax);
        } else {
            output = 0;
        }
        return output;
    }

    @Override
    public void disable() {

    }
}
