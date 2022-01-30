package frc.team2485.WarlordsLib.math;

import edu.wpi.first.math.MathUtil;
import java.util.TreeMap;

/** Linear interpolator */
public class Interpolator {

  /**
   * Linearly interpolates based on an input key and a map of key/value pairs.
   *
   * @param map TreeMap of key/value pairs with which to interpolate
   * @param inputKey the key at which to interpolate
   * @return the interpolated value
   */
  public static double linearInterpolate(TreeMap<Double, Double> map, double inputKey) {

    inputKey = MathUtil.clamp(inputKey, map.firstKey(), map.lastKey());

    // find the points that are the floor and ceiling of the input in the key space of the treemap
    // (the ones right below and right above the input)
    Double lower = map.floorKey(inputKey);
    Double higher = map.ceilingKey(inputKey);

    // this is to prevent dividing by 0
    if (inputKey == higher) {
      return map.get(higher);
    }

    double t = (inputKey - lower) / (higher - lower);
    return MathUtil.interpolate(map.get(lower), map.get(higher), t);
  }
}
