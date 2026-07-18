// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.TurretFlywheelConstants;

class TurretFlywheelMathTest {
    @Test
    void checksWhetherFlywheelIsInsideRequestedVelocityTolerance() {
        assertTrue(TurretFlywheelMath.isWithinVelocityTolerance(42.0, 46.9, 5.0));
        assertFalse(TurretFlywheelMath.isWithinVelocityTolerance(42.0, 47.1, 5.0));
    }

    @Test
    void flywheelReadinessIsLooserThanStrictTargetWindow() {
        assertTrue(TurretFlywheelMath.isWithinVelocityTolerance(
            42.0,
            46.9,
            TurretFlywheelConstants.kReadyVelocityToleranceRotationsPerSecond
        ));
        assertFalse(TurretFlywheelMath.isWithinVelocityTolerance(
            42.0,
            47.1,
            TurretFlywheelConstants.kReadyVelocityToleranceRotationsPerSecond
        ));
        assertFalse(TurretFlywheelMath.isWithinVelocityTolerance(
            42.0,
            45.0,
            TurretFlywheelConstants.kVelocityToleranceRotationsPerSecond
        ));
        assertTrue(TurretFlywheelMath.isWithinVelocityTolerance(
            42.0,
            44.0,
            TurretFlywheelConstants.kVelocityToleranceRotationsPerSecond
        ));
    }

    @Test
    void shootingReadinessKeepsTheExistingMinimumButAllowsUnlimitedOverspeed() {
        assertTrue(TurretFlywheelMath.isAtOrAboveMinimumReadyVelocity(
            42.0,
            34.0,
            8.0
        ));
        assertFalse(TurretFlywheelMath.isAtOrAboveMinimumReadyVelocity(
            42.0,
            33.9,
            8.0
        ));
        assertTrue(TurretFlywheelMath.isAtOrAboveMinimumReadyVelocity(
            42.0,
            80.0,
            8.0
        ));
    }

    @Test
    void appliesFeedingLoadFeedforwardOnlyWhileFeeding() {
        assertTrue(TurretFlywheelMath.getFeedingLoadFeedforwardVolts(
            true,
            TurretFlywheelConstants.kFeedingLoadFeedforwardVolts
        ) > 0.0);
        assertFalse(TurretFlywheelMath.getFeedingLoadFeedforwardVolts(
            false,
            TurretFlywheelConstants.kFeedingLoadFeedforwardVolts
        ) > 0.0);
    }

    @Test
    void combinesShotAndFeedingLoadFeedforward() {
        assertEquals(
            1.0,
            TurretFlywheelMath.getTotalAdditionalFeedforwardVolts(true, 0.5, 0.5),
            1.0e-9
        );
        assertEquals(
            0.5,
            TurretFlywheelMath.getTotalAdditionalFeedforwardVolts(false, 0.5, 0.5),
            1.0e-9
        );
    }
}
