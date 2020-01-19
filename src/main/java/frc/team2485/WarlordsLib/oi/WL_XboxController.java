package frc.team2485.WarlordsLib.oi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * An XboxController with default deadbanding and a convenience function for getting JoystickButtons.
 */
public class WL_XboxController extends XboxController {

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
    public JoystickButton getJoystickButton(Button button) {
        return getJoystickButton(button.value);
    }

    /**
     * Returns JoystickAxisButton
     * @param port port number of axis/joystick
     * @param threshold threshold of JoystickAxis
     * @return JoystickAxisButton
     */
    public JoystickAxisButton getJoystickAxisButton(int port, double threshold) {
        return new JoystickAxisButton(this, port, threshold);
    }

    /**
     * Returns JoystickAxisButton
     * @param axis XboxJoystick joystick axis
     * @param threshold threshold of JoystickAxis
     * @return JoystickAxisButton
     */
    public JoystickAxisButton getJoystickAxisButton(Axis axis, double threshold) {
        return getJoystickAxisButton(axis.value, threshold);
    }

}
