package frc.team2485.WarlordsLib.motorcontrol;

import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.team2485.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.SavableConfigs;

public class SmartMotionSparkMax extends PIDSparkMax {

    private double minVel, maxVel, maxAcc, allowedError;

    private CANPIDController.AccelStrategy accelStrategy;

    /**
     * Create a new Brushless SPARK MAX Controller
     *
     * @param deviceID    The device ID.
     * @param controlType
     */
    public SmartMotionSparkMax(int deviceID, ControlType controlType) {
        super(deviceID, controlType);

        this.maxVel = this.getController().getSmartMotionMaxVelocity(0);
        this.minVel = this.getController().getSmartMotionMinOutputVelocity(0);
        this.maxAcc = this.getController().getSmartMotionMaxAccel(0);
        this.allowedError = this.getController().getSmartMotionAllowedClosedLoopError(0);
        this.accelStrategy = this.getController().getSmartMotionAccelStrategy(0);
    }

    public double getMaxVel() {
        return this.maxVel;
    }

    public void setMaxVel(double maxVel) {
        if (maxVel != this.maxVel) {
            this.getController().setSmartMotionMaxVelocity(maxVel, 0);
            this.maxVel = maxVel;
        }

    }

    public double getMinVel() {
        return this.minVel;
    }

    public void setMinVel(double minVel) {
        if (minVel != this.minVel){
            this.getController().setSmartMotionMinOutputVelocity(minVel, 0);
            this.minVel = minVel;
        }
    }

    public double getMaxAcc() {
        return this.maxAcc;
    }

    public void setMaxAcc(double maxAcc) {
        if (maxAcc != this.maxAcc) {
            this.getController().setSmartMotionMaxAccel(maxAcc, 0);
            this.maxAcc = maxAcc;
        }

    }

    public double getAllowedError() {
        return this.allowedError;
    }

    public void setAllowedError(double allowedError) {
        if (allowedError != this.allowedError) {
            this.getController().setSmartMotionAllowedClosedLoopError(allowedError, 0);
            this.allowedError = allowedError;
        }

    }

    public void setAccelStrategy(CANPIDController.AccelStrategy accelStrategy) {
        this.getController().setSmartMotionAccelStrategy(accelStrategy, 0);
        this.accelStrategy = accelStrategy;
    }

    public CANPIDController.AccelStrategy getAccelStrategy() {
        return this.accelStrategy;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Max velocity", this::getMaxVel, this::setMaxVel);
        builder.addDoubleProperty("Min velocity", this::getMinVel, this::setMinVel);
        builder.addDoubleProperty("Max acceleration", this::getMaxAcc, this::setMaxAcc);
        builder.addDoubleProperty("Allowed error", this::getAllowedError,this::setAllowedError);
    }

    @Override
    public void loadConfigs(LoadableConfigs configs) {
        super.loadConfigs(configs);
        this.setMaxVel(configs.getDouble("maxVel", this.getMaxVel()));
        this.setMinVel(configs.getDouble("minVel", this.getMinVel()));
        this.setMaxAcc(configs.getDouble("maxAcc", this.getMaxAcc()));
        this.setAllowedError(configs.getDouble("allowedError", this.getAllowedError()));
    }

    @Override
    public void saveConfigs(SavableConfigs configs) {
        super.saveConfigs(configs);
        configs.put("maxVel", this.getMaxVel());
        configs.put("minVel", this.getMinVel());
        configs.put("maxAcc", this.getMaxAcc());
        configs.put("allowedError", this.getAllowedError());
    }
}
