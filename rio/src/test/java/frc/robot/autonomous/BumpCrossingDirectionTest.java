// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class BumpCrossingDirectionTest {
    private static final double kTolerance = 1.0e-9;

    @Test
    void preservesRequestedSpeedMagnitudeForDiagonalMotion() {
        double requestedSpeedMetersPerSecond = 4.0;

        for (BumpCrossingDirection direction : BumpCrossingDirection.values()) {
            assertEquals(
                requestedSpeedMetersPerSecond,
                Math.hypot(
                    direction.getVelocityXMetersPerSecond(requestedSpeedMetersPerSecond),
                    direction.getVelocityYMetersPerSecond(requestedSpeedMetersPerSecond)
                ),
                kTolerance
            );
        }
    }

    @Test
    void usesWpilibRobotRelativeDirectionSigns() {
        double speedMetersPerSecond = 1.0;

        assertDirectionSigns(BumpCrossingDirection.FORWARD_LEFT, speedMetersPerSecond, 1.0, 1.0);
        assertDirectionSigns(BumpCrossingDirection.FORWARD_RIGHT, speedMetersPerSecond, 1.0, -1.0);
        assertDirectionSigns(BumpCrossingDirection.BACKWARD_LEFT, speedMetersPerSecond, -1.0, 1.0);
        assertDirectionSigns(BumpCrossingDirection.BACKWARD_RIGHT, speedMetersPerSecond, -1.0, -1.0);
    }

    private void assertDirectionSigns(
        BumpCrossingDirection direction,
        double speedMetersPerSecond,
        double expectedXSign,
        double expectedYSign
    ) {
        assertTrue(
            direction.getVelocityXMetersPerSecond(speedMetersPerSecond) * expectedXSign > 0.0
        );
        assertTrue(
            direction.getVelocityYMetersPerSecond(speedMetersPerSecond) * expectedYSign > 0.0
        );
    }
}
