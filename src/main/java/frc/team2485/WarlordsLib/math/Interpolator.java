package frc.team2485.WarlordsLib.math;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;

/** Linear interpolator */
public class Interpolator {

  private TreeMap<Double, Double> m_points;

  public Interpolator(TreeMap<Double, Double> points) {
    TreeMap<Double,Double> unsortedPoints = points;

    //sort points
    unsortedPoints.entrySet()
    .stream()
    .sorted(Map.Entry.comparingByKey())
    .forEachOrdered(x -> m_points.put(x.getKey(), x.getValue()));

  }

  public double getOutput(double input) {
    input = MathUtil.clamp(input, m_points.firstKey(), m_points.lastKey());

    //find the points that are the floor and ceiling of the input in the key space of the treemap
    //(the ones right below and right above the input)
    Double lower = m_points.floorKey(input);
    Double higher = m_points.ceilingKey(input);

    //interpolate based on these points
    return m_points.get(lower) + 
      ((input - lower) / (higher - lower)) *
        (m_points.get(higher) - m_points.get(lower));
  }
}
