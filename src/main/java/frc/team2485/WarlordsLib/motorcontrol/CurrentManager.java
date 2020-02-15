package frc.team2485.WarlordsLib.motorcontrol;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

import java.util.ArrayList;

public class CurrentManager implements Sendable {
    private double m_maxBatteryCurrent, m_totalCurrentDraw, m_prioritizedTotal, m_deprioritizedTotal;
    private int m_prioritizedCount, m_deprioritizedCount;
    private ArrayList<PrioritizedManageable> m_manageables;

    public CurrentManager(double maxBatteryCurrent) {
        m_maxBatteryCurrent = maxBatteryCurrent;
        m_manageables = new ArrayList<PrioritizedManageable>();
    }

    public void add(Manageable m, boolean priority) {
        if(priority) {
            m_manageables.add(0, new PrioritizedManageable(m, priority));
            m_prioritizedCount++;
        } else {
            m_manageables.add(new PrioritizedManageable(m, priority));
            m_deprioritizedCount++;
        }
//       perhaps later weight code
//        for (int i = 0; i <= m_manageables.size(); i++) {
//            if(i == m_manageables.size()){
//                m_manageables.add(new WeightedManageable(m, weight));
//                break;
//            }
//
//            if(m_manageables.get(i).getWeight() < weight) {
//                m_manageables.add(i, new WeightedManageable(m, weight));
//                break;
//            }
//        }
    }

    /**
     * Updates the current manager and the maximum currents of all manageables (if needed).
     */
    public void update() {
        m_totalCurrentDraw = 0;
        m_prioritizedTotal = 0;
        for (PrioritizedManageable m : m_manageables) { //find total current draw
            double current = m.getManageable().getOutputCurrent();
            if (m.getPriority()) {
                m_prioritizedTotal += current;
            }
            m_totalCurrentDraw += current;
        }

        m_deprioritizedTotal = m_totalCurrentDraw - m_prioritizedTotal;
    }

    public void regulate() {
        if (m_totalCurrentDraw > m_maxBatteryCurrent) { //if we are drawing too much
            double deprioritizedCut; //fraction of deprioritized current that will remain

            if (m_totalCurrentDraw - m_maxBatteryCurrent >= m_prioritizedTotal) { //cutting DP will suffice
                double remainingCurrent = m_maxBatteryCurrent - m_prioritizedTotal; //current remaining after prioritized motors

                deprioritizedCut = remainingCurrent / m_deprioritizedTotal; //fraction of deprioritized current that will remain

            } else { //cutting DP is not enough
                deprioritizedCut = 0; //cut all deprioritized current
                double prioritizedCut = m_maxBatteryCurrent / m_prioritizedTotal; //fraction of prioritized current that will remain

                for (int i = m_deprioritizedCount; i < m_manageables.size(); i++) {
                    Manageable m = m_manageables.get(i).getManageable();
                    m.setAdjustedMaxCurrent(m.getOutputCurrent() * prioritizedCut);
                }
            }
            //will always cut deprioritized
            for(int i = 0; i < m_deprioritizedCount; i++) {
                Manageable m = m_manageables.get(i).getManageable();
                m.setAdjustedMaxCurrent(m.getOutputCurrent() * deprioritizedCut);
            }
        }
    }

    public void run() {
        update();
        regulate();
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
        private Manageable m;
        private boolean priority;
        public PrioritizedManageable (Manageable m, boolean priority) {
            this.m = m;
            this.priority = priority;
        }

        public Manageable getManageable() {
            return m;
        }

        public boolean getPriority() {
            return priority;
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
