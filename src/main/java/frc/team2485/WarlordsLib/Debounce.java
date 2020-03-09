package frc.team2485.WarlordsLib;

/**
 * This class "averages" the values of a noisy boolean sensor.
 */
public class Debounce {

    private boolean m_output;

    private int m_counter;

    private int m_maxDebounceCount;

    /**
     * @param initValue the first value
     * @param maxDebounceCount the number of successive counts to consider a boolean changed
     */
    public Debounce(boolean initValue, int maxDebounceCount) {
        this.m_output = initValue;
        this.m_maxDebounceCount = maxDebounceCount;
        this.m_counter = !initValue ? 0 : m_maxDebounceCount;
    }

    public boolean getNextValue(boolean input) {
        if (!m_output && input) {
            m_counter++;
            if (m_counter >= m_maxDebounceCount) {
                m_output = input;
            }
        } else if (m_output && !input) {
            m_counter--;
            if (m_counter <= -m_maxDebounceCount) {
                m_output = input;
            }

        } else if (input == m_output) {
            m_counter = 0;
        }

        return m_output;
    }



}
