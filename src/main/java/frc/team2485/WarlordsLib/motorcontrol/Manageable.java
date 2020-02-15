package frc.team2485.WarlordsLib.motorcontrol;

public interface Manageable {
    public double getAbsoluteMaxCurrent();
    public void setAdjustedMaxCurrent(double maxCurrent);
    public double getAdjustedMaxCurrent();
    public double getOutputCurrent();
}
