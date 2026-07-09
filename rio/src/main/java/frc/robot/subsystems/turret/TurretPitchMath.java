// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

final class TurretPitchMath {
    private TurretPitchMath() {}

    static boolean isWithinTolerance(
        double targetPitchDegrees,
        double measuredPitchDegrees,
        double toleranceDegrees
    ) {
        return Math.abs(targetPitchDegrees - measuredPitchDegrees) <= toleranceDegrees;
    }
}
