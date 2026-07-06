// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class TurretHeadingMathTest {
    private static final double kMinHeadingDegrees = -330.0;
    private static final double kMaxHeadingDegrees = 150.0;
    private static final double kWrapDegrees = 360.0;
    private static final double kTolerance = 1.0e-9;

    @Test
    void keepsNearbyEquivalentWhenBothChoicesAreLegalNearLowerLimit() {
        assertNearestEquivalent(-330.0, 30.0, -320.0);
    }

    @Test
    void keepsNearbyEquivalentWhenBothChoicesAreLegalInsideOverlap() {
        assertNearestEquivalent(-250.0, 110.0, -250.0);
    }

    @Test
    void keepsUnwrappedRequestWhenItIsCloserToReference() {
        assertNearestEquivalent(30.0, 30.0, 20.0);
    }

    @Test
    void wrapsOnlyWhenCrossingUpperLimit() {
        assertNearestEquivalent(145.0, 145.0, 140.0);
        assertNearestEquivalent(-205.0, 155.0, 145.0);
    }

    @Test
    void wrapsOnlyWhenCrossingLowerLimit() {
        assertNearestEquivalent(-325.0, -325.0, -320.0);
        assertNearestEquivalent(25.0, -335.0, -325.0);
    }

    @Test
    void prefersUnwrappedRequestWhenExactlyBetweenEquivalentChoices() {
        assertNearestEquivalent(90.0, 90.0, -90.0);
    }

    @Test
    void clampsWhenNoEquivalentHeadingIsInsideWindow() {
        assertEquals(
            10.0,
            TurretHeadingMath.chooseNearestEquivalentInWindow(
                20.0,
                0.0,
                -10.0,
                10.0,
                kWrapDegrees
            ),
            kTolerance
        );
    }

    private static void assertNearestEquivalent(
        double expectedHeadingDegrees,
        double requestedHeadingDegrees,
        double referenceHeadingDegrees
    ) {
        assertEquals(
            expectedHeadingDegrees,
            TurretHeadingMath.chooseNearestEquivalentInWindow(
                requestedHeadingDegrees,
                referenceHeadingDegrees,
                kMinHeadingDegrees,
                kMaxHeadingDegrees,
                kWrapDegrees
            ),
            kTolerance
        );
    }
}
