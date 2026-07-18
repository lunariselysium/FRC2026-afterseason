// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

final class TurretFlywheelMath {
    private TurretFlywheelMath() {}

    static boolean isWithinVelocityTolerance(
        double targetVelocityRotationsPerSecond,
        double measuredVelocityRotationsPerSecond,
        double toleranceRotationsPerSecond
    ) {
        return Math.abs(targetVelocityRotationsPerSecond - measuredVelocityRotationsPerSecond)
            <= toleranceRotationsPerSecond;
    }

    static boolean isAtOrAboveMinimumReadyVelocity(
        double targetVelocityRotationsPerSecond,
        double measuredVelocityRotationsPerSecond,
        double underspeedToleranceRotationsPerSecond
    ) {
        return measuredVelocityRotationsPerSecond
            >= targetVelocityRotationsPerSecond - underspeedToleranceRotationsPerSecond;
    }

    static double getFeedingLoadFeedforwardVolts(
        boolean feedingLoadActive,
        double feedingLoadFeedforwardVolts
    ) {
        return feedingLoadActive ? feedingLoadFeedforwardVolts : 0.0;
    }

    static double getTotalAdditionalFeedforwardVolts(
        boolean feedingLoadActive,
        double feedingLoadFeedforwardVolts,
        double shotFeedforwardVolts
    ) {
        return shotFeedforwardVolts + getFeedingLoadFeedforwardVolts(
            feedingLoadActive,
            feedingLoadFeedforwardVolts
        );
    }
}
