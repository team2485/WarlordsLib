package frc.team2485.WarlordsLib.oi;

/**
 * Util class with general static methods for joystick deadbanding and scaling.
 *
 * @author Patrick Wamsley
 * @author Anoushka Bose
 * @author Aidan Fay
 * @author Jeremy McCulloch
 */
public class Deadbander {

    /**
     * Simple deadband with linear scale.
     * @param value input value from joystick/controller
     * @param threshold deadband threshold. Values below this threshold are set to 0.
     * @return deadbanded and linearly scaled input value
     */
    public static double linearScaledDeadband(double value, double threshold) {
        if (Math.abs(value) <= threshold)
            return 0;

        double returnVal = (1 / (1 - threshold)) * (Math.abs(value) - threshold);

        return value > 0 ? returnVal : -returnVal;
    }

    /**
     * Cubic scaling of input
     * @param value input value from joystick/controller
     * @param weight increases sensitivity at lower input values and decreases sensitivity at higher values.
     * @return cubically scaled input
     */
    public static double cubicScale(double value, double weight) {
        return weight * value * value * value + (1.0 - weight) * value;
    }

    /**
     * Thresholds and scales cubically
     * @param value input value from joystick/controller
     * @param threshold deadband threshold. Values below this threshold are set to 0.
     * @param weight increases sensitivity at lower input values and decreases sensitivity at higher values.
     * @return deadbanded and cubically scaled input
     */
    public static double cubicScaledDeadband(double value, double threshold, double weight) {
        if (Math.abs(value) <= threshold) return 0;

        return (cubicScale(value, weight) - (Math.abs(value)/value) * cubicScale(threshold, weight)) / (1.0 - cubicScale(threshold, weight));
    }

    /**
     * Thresholds and scales linearly using linear ramps with potentially different slopes
     * @param val = raw input from controllers/joysticks
     * @param threshold = deadband threshold  (values less than this are ignored)
     * @param absoluteMin = absolute value of min range (must be greater than 0)
     * @param absoluteMidOut = absolute value of what absoluteMidIn should map to
     * @param absoluteMidIn = ramps using one slope when less that absoluteMidIn, and another when greater
     * @param absoluteMax = absolute value of max range (must be less than 1)
     *
     * @return Corrected input from joystick. If input is below threshold, 0 is returned.
     * 		   If not, input is scaled between (min, max) with absoluteMidIn mapping to absoluteMidOut	 */
    public static double deadbandAndScaleDualRamp(double val, double threshold, double absoluteMin, double absoluteMidIn, double absoluteMidOut,
                                                  double absoluteMax) {

        double returnVal;

        if (Math.abs(val) <= threshold) {
            return 0;
        } else if (Math.abs(val) <= absoluteMidIn) {
            returnVal = ((absoluteMidOut - absoluteMin) / (absoluteMidIn - threshold)) * (Math.abs(val) - threshold) + absoluteMin;
        } else {
            returnVal = ((absoluteMax - absoluteMidOut) / (1 - absoluteMidIn)) * (Math.abs(val) - absoluteMidIn) + absoluteMidOut;
        }

        return val > 0 ? returnVal : -returnVal;

    }
}
