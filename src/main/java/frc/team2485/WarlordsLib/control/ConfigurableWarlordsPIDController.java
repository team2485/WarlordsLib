package frc.team2485.WarlordsLib.control;

import frc.team2485.WarlordsLib.robotConfigs.RobotConfigurator;

/**
 * Overrides the set methods
 */
public class ConfigurableWarlordsPIDController extends WarlordsPIDController {

    private String _category;

    public ConfigurableWarlordsPIDController(String category) {
        this(0,0,0, category);
        this._category = category;
    }

    public ConfigurableWarlordsPIDController(double pBackup, double iBackup, double dBackup, String category) {
        super(
                RobotConfigurator.getInstance().getDouble(category, "p", pBackup),
                RobotConfigurator.getInstance().getDouble(category,"i", iBackup),
                RobotConfigurator.getInstance().getDouble(category,"d",dBackup));
        this._category = category;
    }

    public ConfigurableWarlordsPIDController(double pBackup, double iBackup, double dBackup, double period, String category) {
        super(pBackup,iBackup,dBackup, period);
        this._category = category;
    }

    public void setP(double p) {
        RobotConfigurator.getInstance().put(_category,"p", p);
        super.setP(p);
    }

    public void setI(double i) {
        RobotConfigurator.getInstance().put(_category,"i", i);
        super.setI(i);
    }

    public void setD(double d) {
        RobotConfigurator.getInstance().put(_category,"d", d);
        super.setD(d);
    }

    public void setF(double f) {
        RobotConfigurator.getInstance().put(_category,"f", f);
        super.setD(f);
    }

    public void setT(double t) {
        RobotConfigurator.getInstance().put(_category, "t", t);
        super.setT(t);
    }
}
