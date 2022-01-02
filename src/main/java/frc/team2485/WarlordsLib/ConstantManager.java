package frc.team2485.WarlordsLib;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;

public class ConstantManager {
    private int m_acquiredId;

    public ConstantManager(String filepath) {
        File idFile = new File(filepath);

        m_acquiredId = 0; //default (should be competition bot)

        if(idFile.exists()) {
            try {
                BufferedReader reader = new BufferedReader(new FileReader(idFile));
                
                m_acquiredId = Integer.valueOf(reader.readLine());
                
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            DriverStation.reportWarning("ConstantManager: id file not found, using default", false);
        }
    }

    public <T> T select(T... values) {
        try {
            return values[m_acquiredId];
        } catch (ArrayIndexOutOfBoundsException e) {
            return values[0];
        }
    }

}
