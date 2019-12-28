package frc.team2485.WarlordsLib.robotConfigs;

import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public abstract class SaveableBase extends SendableBase {

    private String categoryName;

    private boolean isSavingConstants;

    public SaveableBase() {
        super();
        this.isSavingConstants = false;
    }

    public SaveableBase(boolean addLiveWindow) {
        super(addLiveWindow);
        this.isSavingConstants = false;
    }

    public SaveableBase(String categoryName, boolean isSavingConstants) {
        super();
        this.categoryName = categoryName;
        this.isSavingConstants = true;
    }


    public SaveableBase(String categoryName, boolean isSavingConstants, boolean addLiveWindow) {
        super(addLiveWindow);
        this.categoryName = categoryName;
        this.isSavingConstants = true;
    }

    public String getCategoryName() {
        return categoryName;
    }

    public boolean isSavingConstants() {
        return isSavingConstants;
    }

    public void setSavingConstants(boolean saveConstants) {
        isSavingConstants = saveConstants;
    }

    public void saveConstant(String key, String value) {
        if (isSavingConstants) RobotConfigurator.getInstance().put(categoryName, key, value);
    }

    public void saveConstant(String key, double value) {
        if (isSavingConstants) RobotConfigurator.getInstance().put(categoryName, key, Double.toString(value));
    }

    public void saveConstant(String key, float value) {
        if (isSavingConstants) RobotConfigurator.getInstance().put(categoryName, key, Float.toString(value));
    }

    public void saveConstant(String key, int value) {
        if (isSavingConstants) RobotConfigurator.getInstance().put(categoryName, key, Integer.toString(value));
    }

    public void saveConstant(String key, long value) {
        if (isSavingConstants) RobotConfigurator.getInstance().put(categoryName, key, Long.toString(value));
    }

    public void saveConstant(String key, boolean value) {
        if (isSavingConstants) RobotConfigurator.getInstance().put(categoryName, key, Boolean.toString(value));
    }
}
