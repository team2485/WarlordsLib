package frc.team2485.WarlordsLib.oi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.frc2.buttons.JoystickButton;

/**
 * An XboxController with default deadbanding and a convenience function for getting JoystickButtons.
 */
public class WL_XboxController extends XboxController {

    public static  enum XboxButton {
        kBumperLeft(5),
        kBumperRight(6),
        kStickLeft(9),
        kStickRight(10),
        kA(1),
        kB(2),
        kX(3),
        kY(4),
        kBack(7),
        kStart(8);

        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        private final int value;

        XboxButton(int value) {
            this.value = value;
        }
    }

    public static enum XboxJoystick {
        kXLeft(0),
        kXRight(4),
        kYLeft(1),
        kYRight(5),
        kTriggerLeft(2),
        kTriggerRight(3);

        private final int value;

        XboxJoystick(int value) {
            this.value = value;
        }
    }

    private final double DEFAULT_XBOX_DEADBAND = 0.2;

    /**
     * Construct an instance of a joystick. The joystick index is the USB port on the drivers
     * station.
     *
     * @param port The port on the Driver Station that the joystick is plugged into.
     */
    public WL_XboxController(int port) {
        super(port);
    }

    /**
     * Returns a JoystickButton
     * @param port port number of button
     * @return JoystickButton
     */
    public JoystickButton getJoystickButton(int port) {
        return new JoystickButton(this, port);
    }

    /**
     * Returns JoystickButton
     * @param button XboxButton button
     * @return JoystickButton
     */
    public JoystickButton getJoystickButton(XboxButton button) {
        return getJoystickButton(button.value);
    }

}
