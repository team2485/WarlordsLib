package frc.team2485.WarlordsLib;

import edu.wpi.first.wpilibj.DriverStation;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;

public class RobotConfig {

    private HashMap<String, String> _map;

    public RobotConfig(String filepath) {
        _map = getFileContent(filepath);
    }

    public String getString(String key, String backup) {
        if (_map.containsKey(key)) {
            return _map.get(key);
        }
        reportMissingConstant(key, backup);
        return backup;
    }

    public double getDouble(String key, double backup) {
        if (_map.containsKey(key)) {
            try {
                return Double.parseDouble(_map.get(key));
            } catch (NumberFormatException e) {
                DriverStation.reportWarning("Constant " + key + " not parseable into a double! Returning backup " + backup, false);
                e.printStackTrace();
            }
        }
        reportMissingConstant(key, Double.toString(backup));
        return backup;
    }
    
    public int getInt(String key, int backup) {
        if (_map.containsKey(key)) {
            try {
                return Integer.parseInt(_map.get(key));
            } catch (NumberFormatException e) {
                DriverStation.reportWarning("Constant " + key + " not parseable into a int! Returning backup " + backup, false);
                e.printStackTrace();
            }
        }
        reportMissingConstant(key, Integer.toString(backup));
        return backup;
    }

    private void reportMissingConstant(String key, String backup) {
        DriverStation.reportWarning("Constant " + key + " not found! Returning backup " + backup, false);
    }

    private static HashMap<String, String> getFileContent(String filepath) {
        String row = "";
        HashMap<String, String> map = new HashMap<String, String>();
        if (new File(filepath).isFile()) {
            try (BufferedReader reader = new BufferedReader(new FileReader(filepath))) {

                while ((row = reader.readLine()) != null) {
                    String[] data = row.split(",");
                    if (data.length == 2) {
                        map.put(data[0], data[1]);
                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            DriverStation.reportWarning("Constants file not found!", false);
        }
        return map;
    }
}
