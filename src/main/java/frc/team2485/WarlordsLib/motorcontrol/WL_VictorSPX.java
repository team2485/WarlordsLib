package frc.team2485.WarlordsLib.motorcontrol;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/**
 * Warlords wrapper for VictorSPX with convenience functions.
 */
public class WL_VictorSPX extends WPI_VictorSPX {

    /**
     * Constructor
     *
     * @param deviceNumber [0,62]
     */
    public WL_VictorSPX(int deviceNumber) {
        super(deviceNumber);
        this.configFactoryDefault();
        this.clearStickyFaults();
    }

    /**
     * The given motor controllers will now follow this motor controller.
     */
    public void setFollowers(BaseMotorController follower, BaseMotorController... followers) {
        follower.follow(this);
        for (BaseMotorController m : followers) {
            m.follow(this);
        }
    }
}
