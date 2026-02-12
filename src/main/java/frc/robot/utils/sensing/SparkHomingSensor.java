// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.sensing;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;

/** Class */
public class SparkHomingSensor {
    final double m_positiveWidth;
    final double m_negativeWidth;

    DigitalInput m_dio;
    Debouncer m_debouncer;
    RelativeEncoder m_motorEncoder;

    boolean m_inverted;
    boolean m_isTripped;

    public SparkHomingSensor(int sensorDioPort, boolean inverted, SparkBase motorController, double positiveWidth, double negativeWidth) {
        m_positiveWidth = positiveWidth;
        m_negativeWidth = negativeWidth;

        m_dio = new DigitalInput(sensorDioPort);
        m_debouncer = new Debouncer(0.05, DebounceType.kRising);
        m_motorEncoder = motorController.getEncoder();

        m_inverted = inverted;
        m_isTripped = false;
    }

    public void check()
    {
        boolean hit = m_debouncer.calculate(m_dio.get()^m_inverted);
        if(hit && !m_isTripped)
        {
            double speed = m_motorEncoder.getVelocity();
            if(speed > 0) {
                m_motorEncoder.setPosition(m_positiveWidth);
            } 
            else if(speed < 0) {
                m_motorEncoder.setPosition(m_negativeWidth);
            }
            m_isTripped = true;
        }
        if(!hit && m_isTripped)
        {
            m_isTripped = false;
        }
    }

    static public class Tuner {
        int positivePasses;
        int negativePasses;
        double negativeTotal;
        double positiveTotal;
        SparkHomingSensor m_sensor;

        public Tuner(SparkHomingSensor sensor)
        {
            m_sensor = sensor;
        }

        public void check()
        {
            boolean hit = m_sensor.m_debouncer.calculate(m_sensor.m_dio.get()^m_sensor.m_inverted);
            if(hit && !m_sensor.m_isTripped)
            {
                double speed = m_sensor.m_motorEncoder.getVelocity();
                double position = m_sensor.m_motorEncoder.getPosition();
                if(speed > 0) {
                    positivePasses++;
                    positiveTotal += position;
                } 
                else if(speed < 0) {
                    negativePasses++;
                    negativeTotal += position;
                }
                m_sensor.m_isTripped = true;
            }
            if(!hit && m_sensor.m_isTripped)
            {
                m_sensor.m_isTripped = false;
            }
        }

        public Pair<Double, Double> getResults()
        {
            double posAvg = positiveTotal/(double)positivePasses;
            double negAvg = negativeTotal/(double)negativePasses;

            return new Pair<Double,Double>(posAvg, negAvg);
        }
    }
}
