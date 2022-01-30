package frc.team2485.WarlordsLib.math;

import edu.wpi.first.math.MathUtil;
import java.util.TreeMap;

/** Linear interpolator */
public class Interpolator {

  /**
   * Linearly interpolates based on an input key and a map of key/value pairs.
   *
   * @param points TreeMap of key/value pairs with which to interpolate
   * @param input the key at which to interpolate
   * @return the interpolated value
   */
  public static double linearInterpolate(TreeMap<Double, Double> points, double input) {

    input = MathUtil.clamp(input, points.firstKey(), points.lastKey());

    // find the points that are the floor and ceiling of the input in the key space of the treemap
    // (the ones right below and right above the input)
    Double lower = points.floorKey(input);
    Double higher = points.ceilingKey(input);

    // this is to prevent dividing by 0
    if (input == higher) {
      return points.get(higher);
    }
    // interpolate based on these points
    return points.get(lower)
        + ((input - lower) / (higher - lower)) * (points.get(higher) - points.get(lower));
  }
}
