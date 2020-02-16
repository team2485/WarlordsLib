package frc.team2485.WarlordsLib.motorcontrol.currentmanagement;

public interface Manageable {

    void setAdjustedMaxCurrent(double maxCurrent);

    double getOutputCurrent();
}
