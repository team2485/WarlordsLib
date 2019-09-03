package frc.team2485.WarlordsLib.controls;

public class WarlordsPIDController extends WarlordsController{

    WarlordsPIDController(double P, double I, double D) {
        super(new ProportionalTerm(P), new IntegralTerm(I), new DerivativeTerm(D));
    }

}
