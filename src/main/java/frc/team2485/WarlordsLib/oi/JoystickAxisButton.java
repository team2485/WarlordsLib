package frc.team2485.WarlordsLib.oi;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.frc2.buttons.Button;

/**
 * Makes a joystick trigger like a button using a threshold.
 *
 * @author Nicholas Contreras
 * @author Nathan Sariowan
 */
public class JoystickAxisButton extends Button {
    private double _threshold;
    private GenericHID _joystick;
    private int _port;

    /**
     * 
     * @param joystick  The GenericHID object that has the button (e.g. XboxJoystick, KinectStick,
   *                     etc)
     * @param port      The port number of the joystick for the button.
     * @param threshold The minimum value the joystick must return to trigger a button. If negative, the button will
     *                  trigger when the joystick is below the threshold; otherwise, it will trigger when the joystick
     *                  is above the threshold.
     */
    public JoystickAxisButton(GenericHID joystick, int port, double threshold) {
        super();
        this._threshold = threshold;
        this._joystick = joystick;
        this._port = port;
    }

    /**
     * Returns true if the joystick value is greater than the given threshold.
     * If the _threshold is negative, returns true if joystick value is less than the given threshold.
     */
    @Override
    public boolean get() {
        double joystickVal = this._joystick.getRawAxis(this._port);

        if (_threshold < 0) {
            return joystickVal <= _threshold;
        } else {
            return joystickVal >= _threshold;
        }
    }
}
