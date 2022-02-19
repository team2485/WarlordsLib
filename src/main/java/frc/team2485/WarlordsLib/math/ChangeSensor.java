package frc.team2485.WarlordsLib.math;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

public class ChangeSensor {
  public enum Mode {
    kBidirectional,
    kSpikeOnly,
    kDipOnly
  }

  private final DoubleSupplier m_valueSupplier;
  private final double m_threshold;
  private final ArrayList<Double> m_windowValues = new ArrayList<Double>();
  private final int m_valuesToAverage;
  private final Mode m_mode;

  private double m_lastAverage;
  private double m_currentAverage;

  public ChangeSensor(
      DoubleSupplier valueSupplier, double threshold, int valuesToAverage, Mode mode) {
    m_valueSupplier = valueSupplier;
    m_threshold = threshold;
    m_mode = mode;
    m_valuesToAverage = valuesToAverage;
  }

  public boolean get() {
    boolean hasChanged = false;
    if (m_mode == Mode.kBidirectional) {
      hasChanged = Math.abs((m_currentAverage - m_lastAverage) / m_valuesToAverage) >= m_threshold;
    } else if (m_mode == Mode.kDipOnly) {
      hasChanged = (m_lastAverage - m_currentAverage) / m_valuesToAverage >= m_threshold;
    } else if (m_mode == Mode.kSpikeOnly) {
      hasChanged = (m_currentAverage - m_lastAverage) / m_valuesToAverage >= m_threshold;
    }

    return hasChanged;
  }

  public void update() {
    m_lastAverage = m_windowValues.stream().mapToDouble(i -> i).average().getAsDouble();

    final double currentValue = m_valueSupplier.getAsDouble();

    m_windowValues.add(currentValue);

    if (m_windowValues.size() == m_valuesToAverage + 1) {
      m_windowValues.remove(0);
    }

    m_currentAverage = m_windowValues.stream().mapToDouble(i -> i).average().getAsDouble();
  }
}
