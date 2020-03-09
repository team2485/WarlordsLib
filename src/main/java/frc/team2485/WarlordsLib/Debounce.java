package frc.team2485.WarlordsLib;

public class Debounce {

    private boolean m_output;

    private int m_counter;

    private int m_maxDebounceCounter;

    public Debounce(boolean initValue, int maxDebounceTime) {
        this.m_output = initValue;
        this.m_maxDebounceCounter = maxDebounceTime;
        this.m_counter = !initValue ? 0 : m_maxDebounceCounter;
    }

    public boolean getNextValue(boolean input) {
        if (!m_output && input) {
            m_counter++;
            if (m_counter >= m_maxDebounceCounter) {
                m_output = input;
            }
        } else if (m_output && !input) {
            m_counter--;
            if (m_counter <= - m_maxDebounceCounter) {
                m_output = input;
            }

        } else if (input == m_output) {
            m_counter = 0;
        }

        return m_output;
    }



}
