// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.TurretPitchConstants;

class TurretPitchMathTest {
    @Test
    void checksWhetherPitchIsInsideRequestedTolerance() {
        assertTrue(TurretPitchMath.isWithinTolerance(10.0, 11.9, 2.0));
        assertFalse(TurretPitchMath.isWithinTolerance(10.0, 12.1, 2.0));
    }

    @Test
    void pitchReadinessIsLooserThanControllerSettlingWindow() {
        assertTrue(TurretPitchMath.isWithinTolerance(
            10.0,
            11.9,
            TurretPitchConstants.kPitchReadyToleranceDegrees
        ));
        assertFalse(TurretPitchMath.isWithinTolerance(
            10.0,
            12.1,
            TurretPitchConstants.kPitchReadyToleranceDegrees
        ));
        assertFalse(TurretPitchMath.isWithinTolerance(
            10.0,
            10.75,
            TurretPitchConstants.kPitchToleranceDegrees
        ));
        assertTrue(TurretPitchMath.isWithinTolerance(
            10.0,
            10.5,
            TurretPitchConstants.kPitchToleranceDegrees
        ));
    }
}
