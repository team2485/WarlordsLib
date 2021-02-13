package frc.team2485.WarlordsLib;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.Sendable;

/**
 * This class detects significant spikes or dips in a value. 
 * The system is "armed" once the value goes more extreme than the "arm" value,
 * and once it passes the "trigger" value, it is considered changed. 
 * This prevents repeated detection of a change due to noise, as the value must at first be above the arm to trigger.  
 */

public class ArmAndTrigger implements Sendable{
    private double m_arm;
    private double m_trigger;
    private boolean m_armed;
    private double m_lastInput;
    private boolean m_detectSpike; //true if detecting spike, false if detecting dip

    public ArmAndTrigger(double arm, double trigger, double initValue, boolean detectSpike) {
        m_arm = arm;
        m_trigger = trigger;
        m_lastInput = initValue;
        m_armed = false;
        m_detectSpike = detectSpike;
    }

    public boolean getNextValue(double input) { //input should be a percentage of setpoint
        input = Math.abs(input);
        if(m_detectSpike) {
            if (input < m_arm) { //disarm if below arm value
                m_armed = false;
            } else if (m_lastInput < m_arm && input >= m_arm) {//if just passed arm value, arm
                m_armed = true;
            } 
    
            m_lastInput = input;
    
            if(m_armed && input > m_trigger) {//if armed and above trigger, record a spike/dip
                m_armed = false;
                return true;
            } else {
                return false;
            }

        } else {

            if (input < m_arm) { //disarm if above arm value
                m_armed = false;
            } else if (m_lastInput > m_arm && input <= m_arm) {//if just passed arm value, arm
                m_armed = true;
            } 
    
            m_lastInput = input;
    
            if(m_armed && input < m_trigger) {//if armed and below trigger, record a spike/dip
                m_armed = false;
                return true;
            } else {
                return false;
            }
        }
        
    }

    public boolean isArmed() {
        return m_armed;
    }

    public double getLastInput() {
        return m_lastInput;
    }

    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("armed", this::isArmed, null);
        builder.addDoubleProperty("last input", this::getLastInput, null);

    }



}
