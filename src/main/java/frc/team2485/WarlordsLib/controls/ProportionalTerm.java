package frc.team2485.WarlordsLib.controls;

public class ProportionalTerm extends SingleConstantControlTerm {

    public ProportionalTerm(double constant) {
        super(constant);
    }

    @Override
    public double calculate(WarlordsController controller) {
        return super.getConstant() * controller.getPositionError();
    }

    @Override
    public void reset() {

    }
}
