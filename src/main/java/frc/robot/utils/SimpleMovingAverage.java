package frc.robot.utils;

import java.util.LinkedList;
import java.util.Queue;

public class SimpleMovingAverage {

    private final Queue<Double> m_dataset = new LinkedList<Double>();
    private final int m_period;
    private double m_sum = 0.0;

    public SimpleMovingAverage(int period) {
        m_period = period;
    }

    public void addData(double num) {
        m_sum += num;

        m_dataset.add(num);

        if (m_dataset.size() > m_period) {
            m_sum -= m_dataset.remove();
        }
    }

    public double getMean() {
        return m_sum / m_period;
    }

    public void reset() {
        m_sum = 0.0;
        m_dataset.clear();
    }
}
