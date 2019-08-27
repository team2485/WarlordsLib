package common;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Makes a button from a trigger or joystick.
 * @author Nathan Sariowan
 */
public class TriggerButton extends JoystickButton {

    private double threshold;
    private GenericHID joystick;
    private int port;

    /**
     * 
     * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick,
   *                     etc)
     * @param port The port number of the joystick for the button.
     * @param threshold The minimum value the joystick must return to trigger a button.
     */
    public TriggerButton(GenericHID joystick, int port, double threshold) {
        super(joystick, port);
        this.threshold = threshold;
        this.joystick = joystick;
        this.port = port;
    }

    /**
     * Returns true if the joystick value is greater than the given threshold.
     * If the threshold is negative, returns true if joystick value is less than the given threshold.
     */
    @Override
    public boolean get() {
        double joystickVal = this.joystick.getRawAxis(this.port);

        if (threshold < 0) {
            return joystickVal <= threshold;
        } else {
            return joystickVal >= threshold;
        }
    }
}
