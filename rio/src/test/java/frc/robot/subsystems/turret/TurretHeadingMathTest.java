// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.TurretConstants;
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

    @Test
    void checksWhetherHeadingIsInsideRequestedTolerance() {
        assertTrue(TurretHeadingMath.isWithinTolerance(10.0, 11.9, 2.0));
        assertFalse(TurretHeadingMath.isWithinTolerance(10.0, 12.1, 2.0));
    }

    @Test
    void choosesEncoderRotationNearestReferenceHeading() {
        double headingDegreesPerEncoderRotation = -56.0;

        assertEquals(
            -0.9,
            TurretHeadingMath.chooseNearestEquivalentEncoderRotationsFromForward(
                0.1,
                50.0,
                headingDegreesPerEncoderRotation
            ),
            kTolerance
        );
    }

    @Test
    void keepsEncoderRotationWhenAlreadyNearestReferenceHeading() {
        double headingDegreesPerEncoderRotation = -56.0;

        assertEquals(
            0.1,
            TurretHeadingMath.chooseNearestEquivalentEncoderRotationsFromForward(
                0.1,
                -6.0,
                headingDegreesPerEncoderRotation
            ),
            kTolerance
        );
    }

    @Test
    void shiftsEncoderTurnTowardRequestedHeadingDirection() {
        double headingDegreesPerEncoderRotation = -56.0;

        assertEquals(
            -0.9,
            TurretHeadingMath.shiftEncoderRotationTowardHeadingDirection(
                0.1,
                1.0,
                headingDegreesPerEncoderRotation
            ),
            kTolerance
        );
        assertEquals(
            1.1,
            TurretHeadingMath.shiftEncoderRotationTowardHeadingDirection(
                0.1,
                -1.0,
                headingDegreesPerEncoderRotation
            ),
            kTolerance
        );
    }

    @Test
    void usesMotorGuidanceToResolveAnAbsoluteEncoderWrap() {
        TurretHeadingMath.EncoderUnwrapResult result =
            TurretHeadingMath.chooseEncoderUnwrap(
                0.60,
                -0.40,
                -33.6,
                -56.0
            );

        assertEquals(0.60, result.encoderRotationsFromForward(), kTolerance);
        assertEquals(0.0, result.motorGuidanceErrorDegrees(), kTolerance);
        assertTrue(result.usedMotorGuidance());
    }

    @Test
    void trustsAbsoluteContinuityWhenMotorGuidanceIsAmbiguous() {
        TurretHeadingMath.EncoderUnwrapResult result =
            TurretHeadingMath.chooseEncoderUnwrap(
                0.0,
                0.0,
                -35.0,
                -56.0
            );

        assertEquals(0.0, result.encoderRotationsFromForward(), kTolerance);
        assertEquals(21.0, result.motorGuidanceErrorDegrees(), kTolerance);
        assertFalse(result.usedMotorGuidance());
    }

    @Test
    void convertsRawEncoderRotationsToDegrees() {
        assertEquals(
            171.7,
            TurretHeadingMath.encoderRotationsToDegrees(171.7 / 360.0),
            kTolerance
        );
    }

    @Test
    void calibratesForwardOffsetFromClockwiseNinetyDegreeReference() {
        assertEquals(
            324.53,
            TurretConstants.kForwardEncoderOffsetDegrees,
            kTolerance
        );
    }

    @Test
    void initializesFromClockwiseNinetyDegreeStartupReference() {
        double encoderRotationsFromForward =
            TurretHeadingMath.initializeEncoderRotationsFromKnownHeading(
                183.1 / 360.0,
                TurretConstants.kForwardEncoderOffsetDegrees,
                -90.0,
                -56.0
            );

        assertEquals(
            -90.0,
            encoderRotationsFromForward * -56.0,
            1.0e-3
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
