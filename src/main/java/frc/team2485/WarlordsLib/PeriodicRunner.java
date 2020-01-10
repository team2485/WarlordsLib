package frc.team2485.WarlordsLib;

import edu.wpi.first.wpilibj.Timer;

public class PeriodicRunner {

    private double _seconds;

    private Runnable _runnable;

    private Timer _timer;

    public PeriodicRunner(Runnable runnable, double seconds) {
        this._runnable = runnable;
        this._seconds = seconds;
        this._timer = new Timer();
        _timer.reset();
    }

    public void init() {
        _timer.reset();
        _timer.start();
    }

    public void run() {
        if (_timer.get() > _seconds) {
            _runnable.run();
            _timer.reset();
        }
    }


}
