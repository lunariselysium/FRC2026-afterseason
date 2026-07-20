// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.scoring;

import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.ScoringConstants.ShotCurve;
import frc.robot.scoring.ScoringCalculator.ShotSetpoint;

public class ShotDistanceTuning {
    private static final double kMultiplierRoundingScale = 100.0;

    private double multiplier = 1.0;

    public void incrementMultiplier() {
        multiplier = roundMultiplier(multiplier + ScoringConstants.kShotDistanceMultiplierStep);
    }

    public void decrementMultiplier() {
        multiplier = roundMultiplier(
            Math.max(0.0, multiplier - ScoringConstants.kShotDistanceMultiplierStep)
        );
    }

    public double getMultiplier() {
        return multiplier;
    }

    public double getBelievedDistanceMeters(double measuredDistanceMeters) {
        return measuredDistanceMeters * multiplier;
    }

    public ShotSetpoint evaluateShotSetpoint(
        double measuredDistanceMeters,
        ShotCurve shotCurve,
        boolean allowExtrapolation
    ) {
        return ScoringCalculator.evaluateShotCurve(
            getBelievedDistanceMeters(measuredDistanceMeters),
            shotCurve,
            allowExtrapolation
        );
    }

    private static double roundMultiplier(double multiplier) {
        return Math.round(multiplier * kMultiplierRoundingScale) / kMultiplierRoundingScale;
    }
}
