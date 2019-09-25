package frc.team2485.WarlordsLib.motorcontrol;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * Kinda copied from WPI's except with Control Modes
 */
public class CTRE_SpeedControllerGroup extends SendableBase implements SpeedController {
    private boolean m_isInverted;
    private final BaseMotorController[] m_speedControllers;
    private static int instances;

    private ControlMode m_defaultControlMode;

    public CTRE_SpeedControllerGroup(BaseMotorController speedController, BaseMotorController... speedControllers) {
        this(ControlMode.PercentOutput, speedController, speedControllers);
    }

    public CTRE_SpeedControllerGroup(ControlMode defaultControlMode, BaseMotorController speedController, BaseMotorController... speedControllers) {
        this.m_speedControllers = new BaseMotorController[speedControllers.length + 1];
        m_speedControllers[0] = speedController;
        addChild(speedController);
        for (int i = 0; i < speedControllers.length; i++) {
            m_speedControllers[i + 1] = speedControllers[i];
            addChild(speedControllers[i]);
        }
        instances++;
        setName("CTRE_SpeedControllerGroup", instances);
        m_defaultControlMode = defaultControlMode;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("CTRE Speed Controller");
        builder.setActuator(true);
        builder.setSafeState(this::stopMotor);
        builder.addDoubleProperty("Value", this::get, this::set);
    }

    @Override
    public void set(double speed) {
        this.set(m_defaultControlMode, speed);
    }

    public void set(ControlMode controlMode, double speed) {
        for (BaseMotorController controller : m_speedControllers) {
            controller.set(controlMode, m_isInverted ? -speed : speed);
        }
    }


    @Override
    public double get() {
        return 0;
    }

    @Override
    public void setInverted(boolean isInverted) {

    }

    @Override
    public boolean getInverted() {
        return false;
    }

    @Override
    public void disable() {

    }

    @Override
    public void stopMotor() {

    }

    @Override
    public void pidWrite(double output) {

    }
}
