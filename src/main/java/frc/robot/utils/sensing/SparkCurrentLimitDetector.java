// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.sensing;

import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

/** Add your docs here. */
public class SparkCurrentLimitDetector {
    public enum HardLimitDirection {
        kForward, //Hit forward limit
        kFree, //Limit is not engaged
        kReverse //Reverse limit hit
    }

    private final double m_velocityTolerance;
    private SparkBase m_motorController;
    private double m_tripPoint;
    private Debouncer m_debouncer;
    private HardLimitDirection m_tripDirection;

    public SparkCurrentLimitDetector(SparkBase motorController, double currentTripPoint, double zeroSpeedTolerance) {
        m_motorController = motorController;
        m_tripPoint = currentTripPoint;
        m_velocityTolerance = zeroSpeedTolerance;
        m_debouncer = new Debouncer(0.1, DebounceType.kRising);
        m_tripDirection = HardLimitDirection.kFree;
    }

    public HardLimitDirection check() {
        double motorVelocity = m_motorController.getEncoder().getVelocity();
        boolean limitHit = m_debouncer.calculate(
            (m_motorController.getOutputCurrent() > m_tripPoint) &&
            (MathUtil.isNear(0, motorVelocity, m_velocityTolerance))
        );

        if(limitHit) {
            m_tripDirection = (m_motorController.getAppliedOutput() > 0) ? HardLimitDirection.kForward : HardLimitDirection.kReverse;
        }
        else if((m_tripDirection == HardLimitDirection.kForward && motorVelocity < m_velocityTolerance) ||
                (m_tripDirection == HardLimitDirection.kReverse && motorVelocity > -m_velocityTolerance)){
            m_tripDirection = HardLimitDirection.kFree;
        }

        return m_tripDirection;
    }
}
