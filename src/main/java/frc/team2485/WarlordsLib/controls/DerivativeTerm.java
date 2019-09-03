package frc.team2485.WarlordsLib.controls;

public class DerivativeTerm extends SingleConstantControlTerm{

    public DerivativeTerm(double constant) {
        super(constant);
    }

    @Override
    public double calculate(WarlordsController controller) {
        return super.getConstant() * controller.getVelocityError();
    }

    @Override
    public void reset() {

    }
}
