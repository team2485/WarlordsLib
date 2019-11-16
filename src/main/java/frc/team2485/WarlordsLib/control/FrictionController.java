package frc.team2485.WarlordsLib.control;

public class FrictionController implements Controller {

    private double _Kv;
    private double _Vmax;
    private double _threshold;

    private double m_setpoint;

    public FrictionController(double kV, double vMax, double threshold) {
        this._Kv = kV;
        this._Vmax = vMax;
        this._threshold = threshold;
    }

    public double getSetpoint() {
        return m_setpoint;
    }

    @Override
    public void setSetpoint(double setpoint) {
        this.m_setpoint = setpoint;
    }

    public double getV() {
        return _Kv;
    }

    public void setV(double m_kV) {
        this._Kv = m_kV;
    }

    public double getVmax() {
        return _Vmax;
    }

    public void setVmax(double m_vMax) {
        this._Vmax = m_vMax;
    }

    @Override
    public double calculate(double input) {
        double output;
        if(m_setpoint >= 0.25){
            output = _Kv * ((_Vmax - input) / _Vmax);
        } else {
            output = 0;
        }
        return output;
    }

    @Override
    public void disable() {

    }
}
