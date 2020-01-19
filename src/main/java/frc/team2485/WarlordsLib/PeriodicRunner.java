package frc.team2485.WarlordsLib;

import edu.wpi.first.wpilibj.Timer;

/**
 * A simple class for running things periodically for given times greater than 0.02 seconds.
 */
public class PeriodicRunner {

    private double m_seconds;

    private Runnable m_runnable;

    private Timer m_timer;

    /**
     * Runs a given method periodically by a given amount of seconds.
     * @param runnable the method run
     * @param seconds seconds in between each call.
     */
    public PeriodicRunner(Runnable runnable, double seconds) {
        this.m_runnable = runnable;
        this.m_seconds = seconds;
        this.m_timer = new Timer();
        m_timer.reset();
        m_timer.start();
    }

    /**
     * Run the periodic runner, which will only run the runner every given amount of seconds.
     */
    public void run() {
        if (m_timer.get() > m_seconds) {
            m_runnable.run();
            m_timer.reset();
        }
    }


}
