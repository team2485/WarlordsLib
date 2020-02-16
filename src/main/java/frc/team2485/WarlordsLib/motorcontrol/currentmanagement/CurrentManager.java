package frc.team2485.WarlordsLib.motorcontrol.currentmanagement;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

import java.util.ArrayList;

/**
 * A global singleton current manager.
 */
public class CurrentManager implements Sendable {

    private static volatile CurrentManager m_instance;

    private double m_maxBatteryCurrent;

    private double DEFAULT_MAX_BATTERY_CURRENT = 120;

    private double m_totalCurrentDraw, m_prioritizedTotal, m_deprioritizedTotal;

    private ArrayList<PrioritizedManageable> m_manageables;

    private boolean m_enabled;

    public static CurrentManager getInstance() {
        if (m_instance == null) {
            synchronized (CurrentManager.class) {
                if (m_instance == null) {
                    m_instance = new CurrentManager();
                }
            }
        }
        return m_instance;
    }

    private CurrentManager() {
        m_maxBatteryCurrent = DEFAULT_MAX_BATTERY_CURRENT;
        m_manageables = new ArrayList<PrioritizedManageable>();
        m_enabled = true;
    }

    public void configMaxBatteryCurrent(double maxBatteryCurrent) {
        this.m_maxBatteryCurrent = maxBatteryCurrent;
    }

    public void add(Manageable m, boolean priority, double absoluteMaxCurrent) {

        m_manageables.add(new PrioritizedManageable(m, priority, absoluteMaxCurrent));
//       perhaps later weight code
//        for (int i = 0; i <= m_manageables.size(); i++) {
//            if(i == m_manageables.size()){
//                m_manageables.add(new WeightedManageable(m_manageable, weight));
//                break;
//            }
//
//            if(m_manageables.get(i).getWeight() < weight) {
//                m_manageables.add(i, new WeightedManageable(m_manageable, weight));
//                break;
//            }
//        }
    }

    /**
     * Updates the current manager and the maximum currents of all manageables (if needed).
     */
    private void updateTotalCurrentDraw() {
        m_totalCurrentDraw = 0;
        m_prioritizedTotal = 0;
        for (PrioritizedManageable m : m_manageables) { //find total current draw
            double current = m.getManageable().getOutputCurrent();
            if (m.isPriority()) {
                m_prioritizedTotal += current;
            }
            m_totalCurrentDraw += current;
        }
        m_deprioritizedTotal = m_totalCurrentDraw - m_prioritizedTotal;
    }

    private void regulateCurrent() {
        if (m_totalCurrentDraw > m_maxBatteryCurrent) { //if we are drawing too much
            double deprioritizedProportion; //fraction of deprioritized current that will remain
            double prioritizedProportion;

            if (m_totalCurrentDraw - m_maxBatteryCurrent >= m_prioritizedTotal) { //cutting DP will suffice
                double remainingCurrent = m_maxBatteryCurrent - m_prioritizedTotal; //current remaining after prioritized motors
                deprioritizedProportion = remainingCurrent / m_deprioritizedTotal; //fraction of deprioritized current that will remain
                prioritizedProportion = 1;
            } else { //cutting DP is not enough
                deprioritizedProportion = 0; //cut all deprioritized current
                prioritizedProportion = (m_maxBatteryCurrent - m_deprioritizedTotal) / m_prioritizedTotal; //fraction of prioritized current that will remain
            }

            //will always cut deprioritized
            for(PrioritizedManageable m : m_manageables) {
                if(m.isPriority()) {
                    m.getManageable().setAdjustedMaxCurrent(m.getManageable().getOutputCurrent() * prioritizedProportion);
                } else {
                    m.getManageable().setAdjustedMaxCurrent(m.getManageable().getOutputCurrent() * deprioritizedProportion);
                }
            }
        }
    }

    public void run() {
        updateTotalCurrentDraw();
        regulateCurrent();
    }

    public void setEnabled(boolean enabled) {
        m_enabled = enabled;
        if (!enabled) {
            for(PrioritizedManageable m : m_manageables) {
                m.getManageable().setAdjustedMaxCurrent(m.getAbsoluteMaxCurrent());
            }
        }
    }

    public double getMaxBatteryCurrent() {
        return m_maxBatteryCurrent;
    }

    public double getTotalCurrentDraw() {
        return m_totalCurrentDraw;
    }

    public double getPrioritizedTotal() {
        return m_prioritizedTotal;
    }

    public double getDeprioritizedTotal() {
        return m_deprioritizedTotal;
    }

//    public int getPrioritizedCount() {
//        return m_prioritizedCount;
//    }
//
//    public int getDeprioritizedCount() {
//        return m_deprioritizedCount;
//    }

    private class PrioritizedManageable {

        private Manageable m_manageable;

        private boolean m_priority;

        private double m_absoluteMaxCurrent;

        private PrioritizedManageable (Manageable manageable, boolean priority, double absoluteMaxCurrent) {
            this.m_manageable = manageable;
            this.m_priority = priority;
            this.m_absoluteMaxCurrent = absoluteMaxCurrent;
        }

        private Manageable getManageable() {
            return m_manageable;
        }

        private boolean isPriority() {
            return m_priority;
        }

        private double getAbsoluteMaxCurrent() {
            return m_absoluteMaxCurrent;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Max Battery Current", this::getMaxBatteryCurrent, null);
        builder.addDoubleProperty("Total Current Draw", this::getTotalCurrentDraw, null);
        builder.addDoubleProperty("Prioritized Current Draw", this::getPrioritizedTotal, null);
        builder.addDoubleProperty("Deprioritized Current Draw", this::getMaxBatteryCurrent, null);
    }
}
