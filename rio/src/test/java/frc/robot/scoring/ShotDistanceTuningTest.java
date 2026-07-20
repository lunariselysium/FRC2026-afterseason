// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.scoring;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.ScoringConstants.ShotCurve;
import frc.robot.Constants.ScoringConstants.ShotCurveType;
import frc.robot.Constants.ScoringConstants.ShotMapPoint;
import frc.robot.scoring.ScoringCalculator.ShotSetpoint;

class ShotDistanceTuningTest {
    private static final double kTolerance = 1.0e-9;

    @Test
    void defaultMultiplierLeavesShotDistanceUnchangedForInterpolation() {
        ShotDistanceTuning tuning = new ShotDistanceTuning();

        assertEquals(1.0, tuning.getMultiplier(), kTolerance);
        assertEquals(5.0, tuning.getBelievedDistanceMeters(5.0), kTolerance);
        assertEquals(4.0, tuning.getBelievedDistanceMeters(4.0), kTolerance);
    }

    @Test
    void incrementMultiplierMakesFourMeterShotUseFourPointTwoMeters() {
        ShotDistanceTuning tuning = new ShotDistanceTuning();

        tuning.incrementMultiplier();

        assertEquals(1.05, tuning.getMultiplier(), kTolerance);
        assertEquals(4.2, tuning.getBelievedDistanceMeters(4.0), kTolerance);
    }

    @Test
    void decrementMultiplierMakesFourMeterShotUseThreePointEightMeters() {
        ShotDistanceTuning tuning = new ShotDistanceTuning();

        tuning.decrementMultiplier();

        assertEquals(0.95, tuning.getMultiplier(), kTolerance);
        assertEquals(3.8, tuning.getBelievedDistanceMeters(4.0), kTolerance);
    }

    @Test
    void multiplierStepsStayRoundedToHundredthsAndDoNotBecomeNegative() {
        ShotDistanceTuning tuning = new ShotDistanceTuning();

        for (int i = 0; i < 3; i++) {
            tuning.incrementMultiplier();
        }
        assertEquals(1.15, tuning.getMultiplier(), 0.0);

        for (int i = 0; i < 23; i++) {
            tuning.decrementMultiplier();
        }
        assertEquals(0.0, tuning.getMultiplier(), 0.0);
    }

    @Test
    void incrementedMultiplierIsUsedForShotInterpolation() {
        ShotDistanceTuning tuning = new ShotDistanceTuning();
        ShotCurve curve = new ShotCurve(
            ShotCurveType.INTERPOLATED_MAP,
            ShotCurveType.INTERPOLATED_MAP,
            new ShotMapPoint[] {
                new ShotMapPoint(4.0, 10.0, 25.0),
                new ShotMapPoint(5.0, 20.0, 35.0),
            },
            new double[] {},
            new double[] {}
        );

        tuning.incrementMultiplier();
        ShotSetpoint setpoint = tuning.evaluateShotSetpoint(4.0, curve, false);

        assertEquals(12.0, setpoint.pitchDegrees(), kTolerance);
        assertEquals(27.0, setpoint.flywheelRotationsPerSecond(), kTolerance);
    }
}
