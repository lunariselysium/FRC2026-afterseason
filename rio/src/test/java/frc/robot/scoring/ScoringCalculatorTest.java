// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.scoring;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.OptionalDouble;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.ScoringConstants.ShotCurve;
import frc.robot.Constants.ScoringConstants.ShotCurveType;
import frc.robot.Constants.ScoringConstants.ShotMapPoint;
import frc.robot.scoring.ScoringCalculator.ShotSetpoint;
import frc.robot.scoring.ScoringCalculator.TargetSelectionMode;
import frc.robot.scoring.ScoringCalculator.TargetMode;

class ScoringCalculatorTest {
    private static final double kTolerance = 1.0e-9;

    @Test
    void blueRobotInsideAllianceZoneTargetsBlueHub() {
        ScoringCalculator.ScoringTarget target = ScoringCalculator.calculateTarget(
            new Pose2d(2.0, ScoringConstants.kBlueHubCenterMeters.getY(), Rotation2d.kZero),
            Alliance.Blue,
            OptionalDouble.empty()
        );

        assertEquals(TargetMode.HUB, target.mode());
        assertEquals(ScoringConstants.kBlueHubCenterMeters.getX(), target.fieldPoint().getX(), kTolerance);
        assertEquals(ScoringConstants.kBlueHubCenterMeters.getY(), target.fieldPoint().getY(), kTolerance);
    }

    @Test
    void redRobotInsideAllianceZoneTargetsRedHub() {
        ScoringCalculator.ScoringTarget target = ScoringCalculator.calculateTarget(
            new Pose2d(15.0, ScoringConstants.kRedHubCenterMeters.getY(), Rotation2d.k180deg),
            Alliance.Red,
            OptionalDouble.empty()
        );

        assertEquals(TargetMode.HUB, target.mode());
        assertEquals(ScoringConstants.kRedHubCenterMeters.getX(), target.fieldPoint().getX(), kTolerance);
        assertEquals(ScoringConstants.kRedHubCenterMeters.getY(), target.fieldPoint().getY(), kTolerance);
    }

    @Test
    void robotOutsideAllianceZoneChoosesNearestPassTarget() {
        ScoringCalculator.ScoringTarget blueUpperTarget = ScoringCalculator.calculateTarget(
            new Pose2d(8.0, ScoringConstants.kBlueUpperPassTargetMeters.getY() + 0.5, Rotation2d.kZero),
            Alliance.Blue,
            OptionalDouble.empty()
        );
        ScoringCalculator.ScoringTarget redLowerTarget = ScoringCalculator.calculateTarget(
            new Pose2d(8.0, ScoringConstants.kRedLowerPassTargetMeters.getY() - 0.5, Rotation2d.kZero),
            Alliance.Red,
            OptionalDouble.empty()
        );

        assertEquals(TargetMode.PASS, blueUpperTarget.mode());
        assertEquals(ScoringConstants.kBlueUpperPassTargetMeters.getY(), blueUpperTarget.fieldPoint().getY(), kTolerance);
        assertEquals(TargetMode.PASS, redLowerTarget.mode());
        assertEquals(ScoringConstants.kRedLowerPassTargetMeters.getY(), redLowerTarget.fieldPoint().getY(), kTolerance);
    }

    @Test
    void hubOnlyTargetSelectionIgnoresPassTargetOutsideAllianceZone() {
        ScoringCalculator.ScoringTarget target = ScoringCalculator.calculateTarget(
            new Pose2d(8.0, ScoringConstants.kBlueUpperPassTargetMeters.getY() + 0.5, Rotation2d.kZero),
            new ChassisSpeeds(0.0, 0.0, 0.0),
            0.0,
            Alliance.Blue,
            OptionalDouble.empty(),
            TargetSelectionMode.HUB_ONLY
        );

        assertEquals(TargetMode.HUB, target.mode());
        assertEquals(ScoringConstants.kBlueHubCenterMeters.getX(), target.fieldPoint().getX(), kTolerance);
        assertEquals(ScoringConstants.kBlueHubCenterMeters.getY(), target.fieldPoint().getY(), kTolerance);
    }

    @Test
    void convertsTurretAxisFieldBearingToTurretRelativeHeading() {
        Pose2d robotPose = new Pose2d(
            2.0,
            ScoringConstants.kBlueHubCenterMeters.getY(),
            Rotation2d.fromDegrees(90.0)
        );
        ScoringCalculator.ScoringTarget target = ScoringCalculator.calculateTarget(
            robotPose,
            Alliance.Blue,
            OptionalDouble.empty()
        );
        double expectedFieldBearingDegrees = Math.toDegrees(Math.atan2(
            target.fieldPoint().getY() - target.turretFieldPoint().getY(),
            target.fieldPoint().getX() - target.turretFieldPoint().getX()
        ));
        double expectedTurretHeadingDegrees = MathUtil.inputModulus(
            expectedFieldBearingDegrees - robotPose.getRotation().getDegrees(),
            -180.0,
            180.0
        );

        assertEquals(expectedTurretHeadingDegrees, target.turretHeadingDegrees(), kTolerance);
    }

    @Test
    void visualTrimAppliesToleranceGainClampAndConfiguredSign() {
        assertEquals(0.0, ScoringCalculator.getHubVisualTrimDegrees(OptionalDouble.empty()), kTolerance);
        assertEquals(0.0, ScoringCalculator.getHubVisualTrimDegrees(OptionalDouble.of(0.5)), kTolerance);
        assertEquals(
            ScoringConstants.kHubVisualTrimYawSign
                * ScoringConstants.kHubVisualTrimYawGain
                * 5.0,
            ScoringCalculator.getHubVisualTrimDegrees(OptionalDouble.of(5.0)),
            kTolerance
        );
        assertEquals(8.0, ScoringCalculator.getHubVisualTrimDegrees(OptionalDouble.of(30.0)), kTolerance);
        assertEquals(-8.0, ScoringCalculator.getHubVisualTrimDegrees(OptionalDouble.of(-30.0)), kTolerance);
    }

    @Test
    void hubVisionAssistAddsCorrectionToPoseBasedHeading() {
        Pose2d robotPose = new Pose2d(
            2.0,
            ScoringConstants.kBlueHubCenterMeters.getY() - 0.4,
            Rotation2d.fromDegrees(15.0)
        );
        ScoringCalculator.ScoringTarget poseOnlyTarget = ScoringCalculator.calculateTarget(
            robotPose,
            Alliance.Blue,
            OptionalDouble.empty()
        );
        ScoringCalculator.ScoringTarget assistedTarget = ScoringCalculator.calculateTarget(
            robotPose,
            120.0,
            Alliance.Blue,
            OptionalDouble.of(5.0)
        );

        assertEquals(
            MathUtil.inputModulus(
                poseOnlyTarget.turretHeadingDegrees()
                    + ScoringCalculator.getHubVisualTrimDegrees(OptionalDouble.of(5.0)),
                -180.0,
                180.0
            ),
            assistedTarget.turretHeadingDegrees(),
            kTolerance
        );
    }

    @Test
    void hubVisionAssistInsideDeadbandKeepsPoseBasedHeading() {
        Pose2d robotPose = new Pose2d(
            2.0,
            ScoringConstants.kBlueHubCenterMeters.getY() - 0.4,
            Rotation2d.fromDegrees(15.0)
        );
        ScoringCalculator.ScoringTarget poseOnlyTarget = ScoringCalculator.calculateTarget(
            robotPose,
            Alliance.Blue,
            OptionalDouble.empty()
        );
        ScoringCalculator.ScoringTarget assistedTarget = ScoringCalculator.calculateTarget(
            robotPose,
            120.0,
            Alliance.Blue,
            OptionalDouble.of(0.5)
        );

        assertEquals(
            poseOnlyTarget.turretHeadingDegrees(),
            assistedTarget.turretHeadingDegrees(),
            kTolerance
        );
    }

    @Test
    void predictsPoseFromRobotRelativeTranslationVelocity() {
        Pose2d predictedPose = ScoringCalculator.predictRobotPose(
            new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90.0)),
            new ChassisSpeeds(2.0, 0.0, 0.0),
            0.5
        );

        assertEquals(1.0, predictedPose.getX(), kTolerance);
        assertEquals(3.0, predictedPose.getY(), kTolerance);
        assertEquals(90.0, predictedPose.getRotation().getDegrees(), kTolerance);
    }

    @Test
    void predictsPoseFromRobotRelativeAngularVelocity() {
        Pose2d predictedPose = ScoringCalculator.predictRobotPose(
            new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0)),
            new ChassisSpeeds(0.0, 0.0, Math.PI),
            0.25
        );

        assertEquals(1.0, predictedPose.getX(), kTolerance);
        assertEquals(2.0, predictedPose.getY(), kTolerance);
        assertEquals(75.0, predictedPose.getRotation().getDegrees(), kTolerance);
    }

    @Test
    void convertsRobotRelativeVelocityToFieldRelativeVelocity() {
        Translation2d fieldVelocity = ScoringCalculator.getFieldRelativeVelocityMetersPerSecond(
            new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90.0)),
            new ChassisSpeeds(2.0, 0.0, 0.0)
        );

        assertEquals(0.0, fieldVelocity.getX(), kTolerance);
        assertEquals(2.0, fieldVelocity.getY(), kTolerance);
    }

    @Test
    void compensatesTargetByRobotVelocityDuringShotFlight() {
        Pose2d robotPose = new Pose2d(
            2.0,
            ScoringConstants.kBlueHubCenterMeters.getY(),
            Rotation2d.kZero
        );
        ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds(0.0, 1.0, 0.0);
        double shotTimeOfFlightSeconds = 0.8;

        ScoringCalculator.ScoringTarget target = ScoringCalculator.calculateTarget(
            robotPose,
            robotRelativeSpeeds,
            shotTimeOfFlightSeconds,
            Alliance.Blue,
            OptionalDouble.empty()
        );
        Translation2d expectedCompensatedPoint = new Translation2d(
            ScoringConstants.kBlueHubCenterMeters.getX(),
            ScoringConstants.kBlueHubCenterMeters.getY() - shotTimeOfFlightSeconds
        );
        double expectedFieldBearingDegrees = Math.toDegrees(Math.atan2(
            expectedCompensatedPoint.getY() - target.turretFieldPoint().getY(),
            expectedCompensatedPoint.getX() - target.turretFieldPoint().getX()
        ));

        assertEquals(
            expectedCompensatedPoint.getX(),
            target.compensatedFieldPoint().getX(),
            kTolerance
        );
        assertEquals(
            expectedCompensatedPoint.getY(),
            target.compensatedFieldPoint().getY(),
            kTolerance
        );
        assertEquals(
            target.turretFieldPoint().getDistance(expectedCompensatedPoint),
            target.distanceMeters(),
            kTolerance
        );
        assertEquals(expectedFieldBearingDegrees, target.fieldBearingDegrees(), kTolerance);
    }

    @Test
    void hubShotMapInterpolationBlocksOutOfRangeFeeding() {
        ShotMapPoint[] map = {
            new ShotMapPoint(1.0, 10.0, 25.0),
            new ShotMapPoint(3.0, 20.0, 35.0),
        };

        ShotSetpoint middle =
            ScoringCalculator.interpolateShotSetpoint(2.0, map, false);
        ShotSetpoint tooFar =
            ScoringCalculator.interpolateShotSetpoint(4.0, map, false);

        assertEquals(15.0, middle.pitchDegrees(), kTolerance);
        assertEquals(30.0, middle.flywheelRotationsPerSecond(), kTolerance);
        assertTrue(middle.feedAllowedByDistance());
        assertEquals(20.0, tooFar.pitchDegrees(), kTolerance);
        assertEquals(35.0, tooFar.flywheelRotationsPerSecond(), kTolerance);
        assertFalse(tooFar.feedAllowedByDistance());
    }

    @Test
    void passShotMapAllowsBoundedExtrapolation() {
        ShotMapPoint[] map = {
            new ShotMapPoint(1.0, 10.0, 25.0),
            new ShotMapPoint(3.0, 20.0, 35.0),
        };

        ShotSetpoint setpoint =
            ScoringCalculator.interpolateShotSetpoint(5.0, map, true);

        assertEquals(30.0, setpoint.pitchDegrees(), kTolerance);
        assertEquals(45.0, setpoint.flywheelRotationsPerSecond(), kTolerance);
        assertTrue(setpoint.feedAllowedByDistance());
    }

    @Test
    void interpolatedCurveUsesMapAndBlocksOutsideRange() {
        ShotMapPoint[] shotMap = {
            new ShotMapPoint(1.0, 10.0, 20.0),
            new ShotMapPoint(3.0, 30.0, 40.0),
        };
        ShotCurve curve = new ShotCurve(
            ShotCurveType.INTERPOLATED_MAP,
            ShotCurveType.INTERPOLATED_MAP,
            shotMap,
            new double[] {},
            new double[] {}
        );

        ShotSetpoint middleSetpoint = ScoringCalculator.evaluateShotCurve(2.0, curve, false);
        assertEquals(20.0, middleSetpoint.pitchDegrees(), kTolerance);
        assertEquals(30.0, middleSetpoint.flywheelRotationsPerSecond(), kTolerance);
        assertTrue(middleSetpoint.feedAllowedByDistance());

        ShotSetpoint lowSetpoint = ScoringCalculator.evaluateShotCurve(0.5, curve, false);
        assertEquals(10.0, lowSetpoint.pitchDegrees(), kTolerance);
        assertEquals(20.0, lowSetpoint.flywheelRotationsPerSecond(), kTolerance);
        assertFalse(lowSetpoint.feedAllowedByDistance());
    }

    @Test
    void polynomialCurveUsesIndependentPitchAndFlywheelCoefficients() {
        ShotMapPoint[] shotMap = {
            new ShotMapPoint(1.0, 10.0, 20.0),
            new ShotMapPoint(3.0, 30.0, 40.0),
        };
        ShotCurve curve = new ShotCurve(
            ShotCurveType.POLYNOMIAL,
            ShotCurveType.POLYNOMIAL,
            shotMap,
            new double[] {10.0, 5.0},
            new double[] {25.0, 5.0}
        );

        ShotSetpoint setpoint = ScoringCalculator.evaluateShotCurve(2.0, curve, false);
        assertEquals(20.0, setpoint.pitchDegrees(), kTolerance);
        assertEquals(35.0, setpoint.flywheelRotationsPerSecond(), kTolerance);
        assertTrue(setpoint.feedAllowedByDistance());
    }
}
