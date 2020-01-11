package frc.team2485.WarlordsLib.robotConfigs;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

/**
 * A custom 2d hashmap of constants for the robot. The first key is the category name (eg Subsystem name) and the second key is the constant name.
 */
public class RobotConfigsMap extends HashMap<String, HashMap<String,String>> {

    public void put(String category, String key, String value) {
        key = key.trim();
        if (this.containsKey(category)) {
            this.get(category).put(key, value);
        } else {
            HashMap<String, String> tempMap = new HashMap<String, String>();
            tempMap.put(key, value);
;           this.put(category, tempMap);
        }
    }

    public String get(String category, String key) {
        key = key.trim();
        if (containsKey(category, key)) {
            return this.get(category).get(key);
        }
        return null;
    }

    /**
     * Gets String constant from configs. Returns backup if not found.
     * @param category
     * @param key name of constant
     * @param backup value of backup constant
     */
    public String getStringOrBackup(String category, String key, String backup) {
        if (containsKey(category, key)) {
            return get(category,key);
        }
        reportWarningNotFound(category, key, backup);
        put(category, key, backup);
        return backup;
    }

    /**
     * Gets double constant from configs. Returns backup if unable to parse or not found.
     * @param category
     * @param key name of constant
     * @param backup value of backup constant
     */
    public double getDoubleOrBackup(String category, String key, double backup) {
        if (containsKey(category, key)) {
            try {
                return Double.parseDouble(get(category, key).trim());
            } catch (NumberFormatException e) {
                reportWarning(category, key, Double.toString(backup), " Unable to parse into double");
//                e.printStackTrace();
                return backup;
            }
        }
        reportWarningNotFound(category, key, Double.toString(backup));
        put(category, key, Double.toString(backup));
        return backup;
    }

    public int getIntOrBackup(String category, String key, int backup) {
        if (containsKey(category, key)) {
            try {
                return Integer.parseInt(get(category, key).trim());
            } catch (NumberFormatException e) {
                reportWarning(category, key, Integer.toString(backup), " Unable to parse into int");
//                e.printStackTrace();
                return backup;
            }
        }
        reportWarningNotFound(category, key, Integer.toString(backup));
        put(category, key, Integer.toString(backup));
        return backup;
    }

    /**
     * Gets boolean constant from configs. Returns backup if unable to parse or not found.
     * @param category
     * @param key name of constant
     * @param backup value of backup constant
     */
    public boolean getBooleanOrBackup(String category, String key, boolean backup) {
        if (containsKey(category, key)) {
            try {
                return Boolean.parseBoolean(get(category, key).trim());
            } catch (NumberFormatException e) {
                reportWarning(category, key, Boolean.toString(backup), " Unable to parse into boolean");
//                e.printStackTrace();
                return backup;
            }
        }
        reportWarningNotFound(category, key, Boolean.toString(backup));
        put(category, key, Boolean.toString(backup));
        return backup;
    }

    /**
     * Gets float constant from configs. Returns backup if unable to parse or not found.
     * @param category
     * @param key name of constant
     * @param backup value of backup constant
     */
    public float getFloatOrBackup(String category, String key, float backup) {
        if (containsKey(category, key)) {
            try {
                return Float.parseFloat(get(category, key).trim());
            } catch (NumberFormatException e) {
                reportWarning(category, key, Float.toString(backup), " Unable to parse into float");
//                e.printStackTrace();
                return backup;
            }
        }
        reportWarningNotFound(category, key, Float.toString(backup));
        put(category, key, Float.toString(backup));
        return backup;
    }

    /**
     * Gets int constant from configs. Returns backup if unable to parse or not found.
     * @param category
     * @param key name of constant
     * @param backup value of backup constant
     */
    public long getLongOrBackup(String category, String key, long backup) {
        if (containsKey(category, key)) {
            try {
                return Long.parseLong(get(category, key).trim());
            } catch (NumberFormatException e) {
                reportWarning(category, key, Long.toString(backup), " Unable to parse into long");
//                e.printStackTrace();
                return backup;
            }
        }

        reportWarningNotFound(category, key, Long.toString(backup));
        put(category, key, Long.toString(backup));
        return backup;
    }

    private void reportWarning(String category, String key, String backup, String error) {
        DriverStation.reportWarning("RobotConfigsMap: Error in retrieving (" + category + ", " + key + "): " + error + "! Returning backup: " + backup + ".", false);
    }

    private void reportWarningNotFound(String category, String key, String backup) {
        reportWarning(category, key, backup, "Not Found");
    }

    public boolean containsKey(String category, String key) {
        return this.containsKey(category) && this.get(category).containsKey(key);
    }

    public boolean containsCategory(String category) {
        return this.containsKey(category);
    }

    public Set<String> keySet(String category) {
        if (this.containsKey(category)) {
            return this.get(category).keySet();
        }
        return null;
    }
}
