package frc.team2485.WarlordsLib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardHelper {

    /**
     * Puts a number and returns it back for reuse.
     * @param key SmartDashboard key
     * @param number number
     * @return number
     */
    public static double putNumber(String key, double number) {
        SmartDashboard.putNumber(key, number);
        return number;
    }

    /**
     * Puts a boolean and returns it back for reuse
     * @param key SmartDashboard key
     * @param bool boolean
     * @return boolean
     */
    public static boolean putBoolean(String key, boolean bool) {
        SmartDashboard.putBoolean(key, bool);
        return bool;
    }

    /**
     * Gets a number; puts a number on SmartDashboard if it hasn't been created yet.
     * @param key SmartDashboard key
     * @param backup default double
     * @return double
     */
    public static double getNumber(String key, double backup) {
        if (!SmartDashboard.containsKey(key)) {
            SmartDashboard.putNumber(key, backup);
        }
        return SmartDashboard.getNumber(key, backup);
    }

    /**
     * Gets a number; puts a number on SmartDashboard if it hasn't been created yet. Default of 0.
     * @param key SmartDashboard key
     * @return double
     */
    public static double getNumber(String key) {
        return SmartDashboardHelper.getNumber(key, 0);
    }

    public static boolean getBoolean(String key, boolean backup) {
        if (!SmartDashboard.containsKey(key)) {
            SmartDashboard.putBoolean(key, backup);
        }
        return SmartDashboard.getBoolean(key, backup);
    }
}
