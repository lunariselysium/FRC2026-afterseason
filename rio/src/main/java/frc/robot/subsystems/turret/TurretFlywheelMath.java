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

    static boolean isWithinAsymmetricVelocityTolerance(
        double targetVelocityRotationsPerSecond,
        double measuredVelocityRotationsPerSecond,
        double underspeedToleranceRotationsPerSecond,
        double overspeedToleranceRotationsPerSecond
    ) {
        return measuredVelocityRotationsPerSecond
                >= targetVelocityRotationsPerSecond - underspeedToleranceRotationsPerSecond
            && measuredVelocityRotationsPerSecond
                <= targetVelocityRotationsPerSecond + overspeedToleranceRotationsPerSecond;
    }

    static double getFeedingLoadFeedforwardVolts(
        boolean feedingLoadActive,
        double feedingLoadFeedforwardVolts
    ) {
        return feedingLoadActive ? feedingLoadFeedforwardVolts : 0.0;
    }
}
