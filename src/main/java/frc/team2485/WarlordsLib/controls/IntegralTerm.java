package frc.team2485.WarlordsLib.controls;

public class IntegralTerm extends SingleConstantControlTerm {

    private double m_totalError;

    public IntegralTerm(double constant) {
        super(constant);
        m_totalError = 0;
    }

    @Override
    public double calculate(WarlordsController controller) {
        double kI = super.getConstant();
        if (kI != 0) {
        m_totalError = WarlordsController.clamp(m_totalError + controller.getPositionError() * controller.getPeriod(), controller.getMinOutput() / kI,
                controller.getMaxOutput() / kI);
        }
        return kI * m_totalError;
    }

    @Override
    public void reset() {
        m_totalError = 0;
    }
}
