package frc.team2485.WarlordsLib.controls;

public abstract class SingleConstantControlTerm implements ControlTerm{

    private double m_constant;

    public SingleConstantControlTerm(double constant) {
        this.m_constant = constant;
    }

    public void setConstant(double constant) {
        this.m_constant = constant;
    }

    public double getConstant() {
        return m_constant;
    }
}
