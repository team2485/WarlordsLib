package frc.team2485.WarlordsLib.oi;

/**
 * Util class with general static methods for joystick deadbanding and scaling. Successor of
 * ThresholdHandler
 */
public class Deadband {

  public static double deadband(double value, double threshold) {
    if (value < threshold && value > -threshold) {
      return 0;
    }
    return value;
  }

  /**
   * Simple deadband with linear scale.
   *
   * @param value input value from joystick/controller
   * @param threshold deadband threshold. Values below this threshold are set to 0.
   * @return deadbanded and linearly scaled input value
   */
  public static double linearScaledDeadband(double value, double threshold) {
    if (Math.abs(value) <= threshold) return 0;

    double returnVal = (1 / (1 - threshold)) * (Math.abs(value) - threshold);

    return Math.copySign(returnVal, value);
  }

  /**
   * Square scaling of input
   *
   * @param value input value from joystick/controller
   * @param weight weight of square (lower the weight, wider the curve)
   * @return quadratically scaled input
   */
  public static double squareScale(double value, double weight) {
    return Math.copySign(weight * value * value, value);
  }

  /**
   * Square scaling of input (default weight of 1)
   *
   * @param value input value from joystick/controller
   * @return square scaled input
   */
  public static double squareScale(double value) {
    return squareScale(value, 1.0);
  }

  /**
   * Thresholds and scales quadratically
   *
   * @param value input value from joystick/controller
   * @param threshold deadband threshold. Values below this threshold are set to 0.
   * @param weight weight of scale (lower the weight, wider the curve)
   * @return quadratically scaled input
   */
  public static double squareScaleDeadband(double value, double threshold, double weight) {
    return squareScale(linearScaledDeadband(value, threshold), weight);
  }

  public static double squareScaleDeadband(double value, double threshold) {
    return squareScale(linearScaledDeadband(value, threshold));
  }

  /**
   * Cubic scaling of input
   *
   * @param value input value from joystick/controller
   * @param weight increases sensitivity at lower input values and decreases sensitivity at higher
   *     values.
   * @return cubically scaled input
   */
  public static double cubicScale(double value, double weight) {
    return weight * value * value * value + (1.0 - weight) * value;
  }

  /**
   * Cubic scaling of input (default weight of 0).
   *
   * @param value input value from joystick/controller
   * @return cubically scaled input
   */
  public static double cubicScale(double value) {
    return cubicScale(value, 1.0);
  }

  /**
   * Thresholds and scales cubically
   *
   * @param value input value from joystick/controller
   * @param threshold deadband threshold. Values below this threshold are set to 0.
   * @param weight increases sensitivity at lower input values and decreases sensitivity at higher
   *     values.
   * @return deadbanded and cubically scaled input
   */
  public static double cubicScaledDeadband(double value, double threshold, double weight) {
    if (Math.abs(value) <= threshold) return 0;

    return (cubicScale(value, weight) - (Math.abs(value) / value) * cubicScale(threshold, weight))
        / (1.0 - cubicScale(threshold, weight));
  }

  /**
   * Thresholds and scales cubically
   *
   * @param value input value from joystick/controller
   * @param threshold deadband threshold. Values below this threshold are set to 0.
   * @return deadbanded and cubically scaled input
   */
  public static double cubicScaledDeadband(double value, double threshold) {
    return cubicScaledDeadband(value, threshold, 1.0);
  }
}
