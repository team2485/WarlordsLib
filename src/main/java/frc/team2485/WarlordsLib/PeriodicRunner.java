package frc.team2485.WarlordsLib;

import edu.wpi.first.wpilibj.Timer;

/**
 * A simple class for running things periodically for given times greater than 0.02 seconds.
 */
public class PeriodicRunner {

    private double _seconds;

    private Runnable _runnable;

    private Timer _timer;

    /**
     * Runs a given method periodically by a given amount of seconds.
     * @param runnable the method run
     * @param seconds seconds in between each call.
     */
    public PeriodicRunner(Runnable runnable, double seconds) {
        this._runnable = runnable;
        this._seconds = seconds;
        this._timer = new Timer();
        _timer.reset();
        _timer.start();
    }

    /**
     * Run the periodic runner, which will only run the runner every given amount of seconds.
     */
    public void run() {
        if (_timer.get() > _seconds) {
            _runnable.run();
            _timer.reset();
        }
    }


}
