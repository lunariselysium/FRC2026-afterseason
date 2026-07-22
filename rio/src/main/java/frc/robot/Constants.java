// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    private Constants() {}

    public static final class VisionConstants {
        private VisionConstants() {}

        public static final AprilTagFields kAprilTagField = AprilTagFields.k2026RebuiltAndymark;

        public static final boolean kEnableVisionPoseFusion = true;

        public static final String kLeftFrameCameraName = "left-frame";
        public static final String kRightFrameCameraName = "right-frame";
        public static final String kBackFrameCameraName = "back-frame";
        public static final String kTurretForwardCameraName = "turret-forward";

        public static final boolean kLeftFramePoseUpdatesEnabled = true;
        public static final boolean kRightFramePoseUpdatesEnabled = true;
        public static final boolean kBackFramePoseUpdatesEnabled = true;
        public static final boolean kTurretForwardPoseUpdatesEnabled = false;

        /*
         * Robot-to-camera transforms use meters and radians, with WPILib axes:
         * +X forward, +Y left, +Z up. Camera rotations describe the camera frame
         * at its mounted pose; a forward-facing camera is Rotation3d.kZero.
         *
         * Set kEnableVisionPoseFusion false if these measurements need to be
         * rechecked against the real robot and PhotonVision pose output.
         */
        public static final double kDrivetrainLengthMeters = millimetersToMeters(635.0);
        public static final double kDrivetrainWidthMeters = millimetersToMeters(736.0);

        public static final Transform3d kRobotToLeftFrameCamera =
            new Transform3d(
                new Translation3d(
                    fromRearMillimeters(51.0),
                    fromLeftMillimeters(35.0),
                    millimetersToMeters(194.8)
                ),
                new Rotation3d(0.0, -Units.degreesToRadians(28.0), Math.PI / 2.0)
            );
        public static final Transform3d kRobotToRightFrameCamera =
            new Transform3d(
                new Translation3d(
                    fromRearMillimeters(261.4),
                    fromRightMillimeters(11.0),
                    millimetersToMeters(367.29)
                ),
                new Rotation3d(0.0, -Units.degreesToRadians(24.99), -Math.PI / 2.0)
            );
        public static final Transform3d kRobotToBackFrameCamera =
            new Transform3d(
                new Translation3d(
                    fromRearMillimeters(18.17),
                    fromLeftMillimeters(308.8),
                    millimetersToMeters(335.0)
                ),
                new Rotation3d(0.0, -Units.degreesToRadians(25.366), Math.PI)
            );

        /*
         * Turret camera geometry is split around the yaw axis so robot code can
         * recompute the transform as the turret rotates. The second transform is
         * measured from the turret yaw axis to the camera lens when turret heading
         * is zero.
         */
        public static final Transform3d kRobotToTurretYawAxis =
            new Transform3d(
                new Translation3d(
                    fromRearMillimeters(183.50),
                    fromLeftMillimeters(181.05),
                    millimetersToMeters(360.22)
                ),
                Rotation3d.kZero
            );
        public static final Transform3d kTurretYawAxisToTurretForwardCamera =
            new Transform3d(
                new Translation3d(
                    millimetersToMeters(144.12),
                    0.0,
                    millimetersToMeters(156.56)
                ),
                new Rotation3d(0.0, -Units.degreesToRadians(34.11), 0.0)
            );
        public static final double kTurretHeadingToRobotYawSign = 1.0;
        public static final double kTurretForwardCameraZeroYawOffsetDegrees = 0.0;

        public static final double kMaxAcceptedPoseZErrorMeters = 0.35;
        public static final double kAcceptedFieldBoundaryMarginMeters = 0.75;
        public static final double kMaxSingleTagAmbiguity = 0.20;
        public static final double kMaxAverageTagDistanceMeters = 7.0;

        public static final double kBaseTranslationStdDevMeters = 0.08;
        public static final double kBaseRotationStdDevRadians = Units.degreesToRadians(15.0);
        public static final double kSingleTagStdDevMultiplier = 3.0;
        public static final double kTurretCameraStdDevMultiplier = 1.5;

        public static final int kRelocalizationRequiredSampleCount = 3;
        public static final int kRelocalizationMinimumTagCount = 1;
        public static final double kRelocalizationMaxPoseZErrorMeters = 0.20;
        public static final double kRelocalizationMaxTranslationSpreadMeters = 0.25;
        public static final double kRelocalizationMaxRotationSpreadRadians =
            Units.degreesToRadians(8.0);

        private static double millimetersToMeters(double millimeters) {
            return millimeters / 1000.0;
        }

        private static double fromRearMillimeters(double millimetersFromRear) {
            return -kDrivetrainLengthMeters / 2.0 + millimetersToMeters(millimetersFromRear);
        }

        private static double fromLeftMillimeters(double millimetersFromLeft) {
            return kDrivetrainWidthMeters / 2.0 - millimetersToMeters(millimetersFromLeft);
        }

        private static double fromRightMillimeters(double millimetersFromRight) {
            return -kDrivetrainWidthMeters / 2.0 + millimetersToMeters(millimetersFromRight);
        }
    }

    public static final class BumpCrossingConstants {
        private BumpCrossingConstants() {}

        /*
         * Initial robot-relative bump-crossing values. Direction is selected by
         * the PathPlanner named command.
         */
        public static final double kCrossingSpeedMetersPerSecond = 2.5;
        public static final double kCrossingDriveTimeSeconds = 1.8;
        public static final double kMaximumDriveTimeSeconds = 1.8;
        public static final double kRelocalizationWarningSeconds = 1.2;
    }

    public static final class DriveConstants {
        private DriveConstants() {}

        public static final double kNormalSupplyCurrentLimitAmps = 25.0;
        public static final double kShootingSupplyCurrentLimitAmps = 2.0;
    }

    public static final class OperatorConstants {
        private OperatorConstants() {}

        public static final int kPrimaryControllerPort = 0;
        public static final int kBackupControllerPort = 1;
        public static final double kShiftRumbleStrength = 0.8;
        public static final double kShiftWarningPulseSeconds = 0.10;
        public static final double kShiftWarningGapSeconds = 0.08;
        public static final double kShiftEndPulseSeconds = 0.35;
        public static final double kJamWarningRumbleStrength = 0.8;
    }

    public static final class AutoConstants {
        private AutoConstants() {}

        /*
         * These controller gains intentionally start at zero because this robot
         * has not supplied characterized PathPlanner translation/rotation gains.
         * Tune them before relying on closed-loop path accuracy.
         */
        public static final double kPathTranslationKp = 8.0;
        public static final double kPathTranslationKi = 0.0;
        public static final double kPathTranslationKd = 0.0;
        public static final double kPathRotationKp = 12.0;
        public static final double kPathRotationKi = 0.0;
        public static final double kPathRotationKd = 0.0;
    }

    public static final class ScoringConstants {
        private ScoringConstants() {}

        public record ShotMapPoint(
            double distanceMeters,
            double pitchDegrees,
            double flywheelRotationsPerSecond
        ) {}

        public enum ShotCurveType {
            INTERPOLATED_MAP,
            POLYNOMIAL
        }

        public record ShotCurve(
            ShotCurveType pitchCurveType,
            ShotCurveType flywheelCurveType,
            ShotMapPoint[] shotMap,
            double[] pitchPolynomialCoefficients,
            double[] flywheelPolynomialCoefficients
        ) {}

        public static final double kFieldLengthMeters = 16.518;
        public static final double kFieldWidthMeters = 8.043;

        public static final Translation2d kBlueHubCenterMeters =
            new Translation2d(4.52265415, 4.0213534);
        public static final Translation2d kRedHubCenterMeters =
            new Translation2d(11.9903494, 4.0213534);

        public static final int[] kBlueHubTagIds = {18, 19, 20, 21, 24, 25, 26, 27};
        public static final int[] kRedHubTagIds = {2, 3, 4, 5, 8, 9, 10, 11};

        public static final double kAllianceZoneDepthMeters = Units.inchesToMeters(182.11);
        public static final double kPassTargetInsetIntoAllianceZoneMeters = Units.inchesToMeters(38.0);
        public static final double kPassTargetYOffsetFromFieldCenterMeters = Units.inchesToMeters(80.0);

        public static final double kBluePassTargetXMeters =
            kAllianceZoneDepthMeters - kPassTargetInsetIntoAllianceZoneMeters;
        public static final double kRedPassTargetXMeters =
            kFieldLengthMeters - kAllianceZoneDepthMeters + kPassTargetInsetIntoAllianceZoneMeters;
        public static final Translation2d kBlueLowerPassTargetMeters =
            new Translation2d(
                kBluePassTargetXMeters,
                kFieldWidthMeters / 2.0 - kPassTargetYOffsetFromFieldCenterMeters
            );
        public static final Translation2d kBlueUpperPassTargetMeters =
            new Translation2d(
                kBluePassTargetXMeters,
                kFieldWidthMeters / 2.0 + kPassTargetYOffsetFromFieldCenterMeters
            );
        public static final Translation2d kRedLowerPassTargetMeters =
            new Translation2d(
                kRedPassTargetXMeters,
                kFieldWidthMeters / 2.0 - kPassTargetYOffsetFromFieldCenterMeters
            );
        public static final Translation2d kRedUpperPassTargetMeters =
            new Translation2d(
                kRedPassTargetXMeters,
                kFieldWidthMeters / 2.0 + kPassTargetYOffsetFromFieldCenterMeters
            );

        public static final double kHubVisualTrimYawSign = 1.0;
        public static final double kHubVisualTrimYawGain = 0.50;
        public static final double kHubVisualTrimToleranceDegrees = 2.0;
        public static final double kHubVisualTrimMaxCorrectionDegrees = 8.0;
        public static final double kHubVisionAssistStaleSeconds = 0.10;

        public static final int kReadyDebounceCycles = 5;
        // Nominal 20 ms score-command cycles.
        public static final int kFlywheelReducedFeedCycles = 5;
        public static final int kFlywheelResumeReadyCycles = 2;
        public static final double kReducedFeedOutputScale = 0.60;
        public static final double kShotMotionPredictionSeconds = 0.12;
        public static final double kShotTimeOfFlightSeconds = 1.0;

        public static final double kMinShotPitchDegrees = TurretPitchConstants.kMinPitchDegrees;
        public static final double kMaxShotPitchDegrees = TurretPitchConstants.kMaxPitchDegrees;
        public static final double kMinShotFlywheelRotationsPerSecond = 20.0;
        public static final double kMaxShotFlywheelRotationsPerSecond = 60.0;
        public static final double kShotDistanceMultiplierStep = 0.05;

        // Hub map is measured through 4.63 m; the 5.60 m endpoint bounds polynomial extrapolation.
        // Pass map is provisional until calibrated.
        public static final ShotMapPoint[] kHubShotMap = {
            new ShotMapPoint(1.74, 4.8, 40.0),
            new ShotMapPoint(2.30, 9.8, 40.0),
            new ShotMapPoint(3.00, 17.0, 43.0),
            new ShotMapPoint(3.90, 20.0, 47.0),
            new ShotMapPoint(4.63, 20.0, 50.0),
            new ShotMapPoint(5.60, 15.9102534776, 56.322988138),
        };
        public static final ShotMapPoint[] kPassShotMap = {
            new ShotMapPoint(2.3, 35, 32),
            new ShotMapPoint(3.33, 35, 34.3),
            new ShotMapPoint(4.5, 35, 38),
            new ShotMapPoint(6.17, 35, 48),
        };
        public static final ShotCurve kHubShotCurve = new ShotCurve(
            ShotCurveType.POLYNOMIAL,
            ShotCurveType.POLYNOMIAL,
            kHubShotMap,
            new double[] {-25.23417318, 21.39877651, -2.50920669},
            new double[] {38.43900829, -0.50053489, 0.6596612}
        );
        public static final ShotCurve kPassShotCurve = new ShotCurve(
            ShotCurveType.INTERPOLATED_MAP,
            ShotCurveType.POLYNOMIAL,
            kPassShotMap,
            new double[] {},
            new double[] {33.86368896, -2.57332096, 0.78704293}
        );
    }

    public static final class MusicConstants {
        private MusicConstants() {}

        public static final String kMusicFileName = "imperialmarch.chrp";

        public record MusicTrackAssignment(int motorCanId, int trackNumber) {}

        public static final MusicTrackAssignment[] kMusicTrackAssignments = {
            new MusicTrackAssignment(8, 0),
            new MusicTrackAssignment(7, 1),
            new MusicTrackAssignment(1, 2),
            new MusicTrackAssignment(2, 3),
            new MusicTrackAssignment(5, 4),
            new MusicTrackAssignment(6, 5),
            new MusicTrackAssignment(4, 6),
            new MusicTrackAssignment(3, 7),
            new MusicTrackAssignment(11, 8),
            new MusicTrackAssignment(12, 9),
            new MusicTrackAssignment(13, 10),
            new MusicTrackAssignment(21, 11),
            new MusicTrackAssignment(22, 12),
            new MusicTrackAssignment(23, 13),
            new MusicTrackAssignment(24, 10),
            new MusicTrackAssignment(31, 4),
            new MusicTrackAssignment(32, 5),
            new MusicTrackAssignment(33, 8),
            new MusicTrackAssignment(34, 10),
            new MusicTrackAssignment(35, 13),
        };
    }

    public static final class TurretConstants {
        private TurretConstants() {}

        public static final int kThroughBoreEncoderDioChannel = 9;

        // REV Through Bore Encoder PWM output: 1 us to 1024 us high in a 1025 us period.
        public static final double kThroughBoreMinDutyCycle = 1.0 / 1025.0;
        public static final double kThroughBoreMaxDutyCycle = 1024.0 / 1025.0;

        public static final int kHeadingMotorCanId = 32;
        public static final String kHeadingMotorCanBus = "canivores";

        public static final double kHeadingMotorPinionTeeth = 12.0;
        public static final double kFirstStageDrivenGearTeeth = 60.0;
        public static final double kIntermediatePulleyTeeth = 18.0;
        public static final double kEncoderShaftPulleyTeeth = 28.0;
        public static final double kEncoderGearTeeth = 14.0;
        public static final double kTurretGearTeeth = 90.0;

        /*
         * Tune this to the raw encoder degrees reported when the turret is pointed
         * straight forward. Because the encoder is on the 14 tooth gear, this offset
         * only disambiguates correctly when the turret starts within about 28 degrees
         * of kStartupTurretHeadingDegrees.
         */
        // Calibrated from the raw 183.1 degree reading at a known -90 degree heading.
        public static final double kForwardEncoderOffsetDegrees = 324.53;
        // The turret must be physically here when robot code starts.
        public static final double kStartupTurretHeadingDegrees = -90.0;

        public static final double kMotorToEncoderShaftReduction =
            (kFirstStageDrivenGearTeeth / kHeadingMotorPinionTeeth)
                * (kEncoderShaftPulleyTeeth / kIntermediatePulleyTeeth);
        public static final double kEncoderShaftToTurretReduction =
            kTurretGearTeeth / kEncoderGearTeeth;
        public static final double kMotorToTurretReduction =
            kMotorToEncoderShaftReduction * kEncoderShaftToTurretReduction;

        // Flip this if positive encoder motion reports the turret heading backwards.
        public static final double kTurretHeadingSign = -1.0;

        // Flip this if positive Kraken encoder motion reports turret heading backwards.
        public static final double kHeadingMotorPositionSign = -1.0;

        public static final double kMinTurretHeadingDegrees = -390.0;
        public static final double kMaxTurretHeadingDegrees = 150.0;
        public static final double kTurretHeadingWrapDegrees = 360.0;
        public static final double kHeadingEncoderStartupGraceSeconds = 1.0;

        public static final double kTargetHeadingStepDegrees = 30.0;
        public static final double kHeadingToleranceDegrees = 0.5;
        public static final double kHeadingReadyToleranceDegrees = 10.0;
        public static final double kMaxTurretHeadingVelocityDegreesPerSecond = 500.0;
        public static final double kMaxTurretHeadingAccelerationDegreesPerSecondSquared = 800.0;
        public static final double kTurretHeadingKp = 0.0044;
        public static final double kTurretHeadingKv = 0.0010;
        public static final double kTurretHeadingKs = 0.015;
        public static final double kMaxTurretMotorOutput = 0.7;
        public static final double kTurretHeadingSupplyCurrentLimitAmps = 10.0;
        public static final double kTurretHeadingStatorCurrentLimitAmps = 20.0;
        public static final double kSysIdQuasistaticRampRateVoltsPerSecond = 0.25;
        public static final double kSysIdDynamicStepVolts = 1.50;
        public static final double kSysIdTimeoutSeconds = 10.0;

        // Flip this if the turret moves away from the target heading.
        public static final double kHeadingMotorOutputSign = -1.0;
    }

    public static final class TurretPitchConstants {
        private TurretPitchConstants() {}

        public static final int kPitchMotorCanId = 33;
        public static final String kPitchMotorCanBus = "canivores";

        public static final double kPitchGearboxReduction = 5.0;
        public static final double kPitchRightAngleReduction = 1.0;
        public static final double kPitchMotorToPinionReduction =
            kPitchGearboxReduction * kPitchRightAngleReduction;

        public static final double kPitchPinionTeeth = 12.0;
        public static final double kPitchPinionRootDiameterMillimeters = 16.08;
        public static final double kPitchPinionOuterDiameterMillimeters = 23.07;
        public static final double kPitchPinionPitchDiameterMillimeters =
            (kPitchPinionRootDiameterMillimeters + kPitchPinionOuterDiameterMillimeters) / 2.0;

        public static final double kPitchRackOuterRadiusMillimeters = 200.27;
        public static final double kPitchRackToothDepthMillimeters = 4.18;
        public static final double kPitchRackPitchRadiusMillimeters =
            kPitchRackOuterRadiusMillimeters - kPitchRackToothDepthMillimeters / 2.0;

        public static final double kPitchDegreesPerMotorRotation =
            180.0
                * kPitchPinionPitchDiameterMillimeters
                / (kPitchMotorToPinionReduction * kPitchRackPitchRadiusMillimeters);

        public static final double kMinPitchDegrees = 0.0;
        public static final double kMaxPitchDegrees = 35.0;
        public static final double kDefaultPitchDegrees = 5.0;
        public static final double kTargetPitchStepDegrees = 5.0;
        public static final double kPitchToleranceDegrees = 0.5;
        public static final double kPitchReadyToleranceDegrees = 2.0;

        public static final double kMaxPitchVelocityDegreesPerSecond = 600.0;
        public static final double kMaxPitchAccelerationDegreesPerSecondSquared = 1000.0;
        public static final double kPitchKp = 0.038;
        public static final double kPitchKv = 0.0012;
        public static final double kPitchKg = 0.002;
        public static final double kPitchGravityOffsetDegrees = 0.0;
        public static final double kMaxPitchMotorOutput = 0.80;

        public static final double kPitchSupplyCurrentLimitAmps = 20.0;
        public static final double kPitchStatorCurrentLimitAmps = 30.0;
        public static final double kPitchHomingMotorOutput = 0.04;
        public static final double kPitchHomingCurrentThresholdAmps = 8.0;
        public static final double kPitchHomingMinRunTimeSeconds = 0.25;
        public static final double kPitchHomingCurrentDebounceSeconds = 0.10;
        public static final double kPitchHomingTimeoutSeconds = 10.0;
        public static final double kSysIdQuasistaticRampRateVoltsPerSecond = 0.20;
        public static final double kSysIdDynamicStepVolts = 2.00;
        public static final double kSysIdTimeoutSeconds = 10.0;

        // Flip if positive motor rotations report pitch downward.
        public static final double kPitchPositionSign = 1.0;

        // Flip if positive mechanism output moves pitch downward.
        public static final double kPitchMotorOutputSign = 1.0;
    }

    public static final class TurretFlywheelConstants {
        private TurretFlywheelConstants() {}

        public static final int kLeaderMotorCanId = 34;
        public static final int kFollowerMotorCanId = 35;
        public static final String kMotorCanBus = "canivores";

        public static final double kTargetVelocityRotationsPerSecond = 42.0;
        public static final double kVelocityToleranceRotationsPerSecond = 2.0;
        public static final double kReadyVelocityToleranceRotationsPerSecond = 5.0;
        public static final double kReadyAdditionalUnderspeedToleranceRotationsPerSecond = 3.0;
        public static final double kFlywheelKp = 0.16;
        public static final double kFlywheelKv = 0.11285;
        public static final double kFlywheelKa = 0.0028782;
        public static final double kFlywheelKs = 0.10952;
        public static final double kFeedingLoadFeedforwardVolts = 0.5;
        // Start pass compensation at the already proven feeding-load voltage.
        public static final double kPassShotFeedforwardVolts = kFeedingLoadFeedforwardVolts;
        public static final double kMotionMagicAccelerationRotationsPerSecondSquared = 70.0;
        public static final double kMotionMagicJerkRotationsPerSecondCubed = 700.0;
        public static final double kSupplyCurrentLimitAmps = 40.0;
        public static final double kStatorCurrentLimitAmps = 80.0;
        public static final double kSysIdQuasistaticRampRateVoltsPerSecond = 0.50;
        public static final double kSysIdDynamicStepVolts = 4.0;
        public static final double kSysIdTimeoutSeconds = 10.0;

        public static final boolean kFollowerOpposesLeader = true;
    }

    public static final class TurretSerializerConstants {
        private TurretSerializerConstants() {}

        public static final int kMotorCanId = 31;
        public static final String kMotorCanBus = "canivores";

        public static final double kMotorOutput = 0.48;
        public static final double kMotorOutputSign = 1.0;
        public static final double kSupplyCurrentLimitAmps = 20.0;
        public static final double kStatorCurrentLimitAmps = 30.0;
    }

    public static final class FeederConstants {
        private FeederConstants() {}

        public static final int kFloorMotorCanId = 21;
        public static final int kHandoffWheelMotorCanId = 22;
        public static final int kBeltLeaderMotorCanId = 23;
        public static final int kBeltFollowerMotorCanId = 24;
        public static final String kMotorCanBus = "canivores";

        public static final double kFloorMotorOutput = 0.95;
        public static final double kBeltMotorOutput = 0.95;
        public static final double kHandoffWheelMotorOutput = 0.95;

        public static final double kFloorMotorOutputSign = 1.0;
        public static final double kBeltMotorOutputSign = 1.0;
        public static final double kHandoffWheelMotorOutputSign = -1.0;
        public static final double kSupplyCurrentLimitAmps = 20.0;
        public static final double kStatorCurrentLimitAmps = 30.0;

        public static final double kJamMinimumCommandedOutput = 0.20;
        public static final double kJamCurrentThresholdAmps =
            kStatorCurrentLimitAmps * 1.00;
        public static final double kJamVelocityThresholdRotationsPerSecond = 1.0;
        public static final double kJamQualificationSeconds = 0.20;
        public static final double kManualUnjamReverseSeconds = 0.20;

        public static final boolean kBeltFollowerOpposesLeader = false;
    }

    public static final class IntakeConstants {
        private IntakeConstants() {}

        public static final int kDeployMotorCanId = 11;
        public static final int kLeftRollerMotorCanId = 12;
        public static final int kRightRollerMotorCanId = 13;
        // Set true if the left roller motor is reinstalled as a follower.
        public static final boolean kLeftRollerMotorPresent = false;
        public static final String kMotorCanBus = "canivores";

        public static final double kMotorPulleyTeeth = 18.0;
        public static final double kDrivenPulleyTeeth = 28.0;
        public static final double kRackPinionGearTeeth = 27.0;

        public static final double kDeployedSetpointMotorRotations = 20.0;
        public static final double kAutoScoreRetractionDeployedFraction = 0.50;
        public static final double kAutoScoreRetractionSetpointMotorRotations =
            kDeployedSetpointMotorRotations * kAutoScoreRetractionDeployedFraction;
        public static final double kAutoScoreOscillationDeployedFraction = 0.80;
        public static final double kAutoScoreOscillationSetpointMotorRotations =
            kDeployedSetpointMotorRotations * kAutoScoreOscillationDeployedFraction;
        public static final double kManualUnjamOutwardTravelFraction = 0.20;
        public static final double kAutoScoreRetractionDelaySeconds = 1.0;
        public static final double kAutoScoreOscillationPhaseSeconds = 1.0;
        public static final double kPositionToleranceMotorRotations = 0.5;
        public static final double kPositionClosedLoopKp = 1.2;
        public static final double kPositionClosedLoopKd = 0.0;
        public static final double kPositionClosedLoopKs = 0.333;
        public static final double kPositionClosedLoopKv = 0.109;
        public static final double kPositionClosedLoopKa = 0.006;
        // CTRE motor-output voltage; positive retracts/stows this mechanism.
        public static final double kStowAssistFeedForwardVolts = 0.5;
        public static final double kMotionMagicCruiseVelocityMotorRotationsPerSecond = 72.0;
        public static final double kMotionMagicAccelerationMotorRotationsPerSecondSquared = 144.0;
        public static final double kAutoScoreRetractionCruiseVelocityMotorRotationsPerSecond = 10.4;
        public static final double kAutoScoreRetractionAccelerationMotorRotationsPerSecondSquared = 20.8;
        public static final double kDeployedHardstopCaptureCurrentThresholdAmps = 7.0;
        public static final double kDeployedHardstopCaptureWindowMotorRotations = 1.2;

        // Positive mechanism position is deploying; positive raw motor position retracts.
        public static final double kDeployPositionSensorSign = -1.0;

        // Positive mechanism position is deploying; positive motor output retracts.
        public static final double kPositionMotorOutputSign = -1.0;
        public static final double kDeploySupplyCurrentLimitAmps = 14.0;
        public static final double kDeployStatorCurrentLimitAmps = 14.0;
        public static final double kDeployHomingSupplyCurrentLimitAmps = 30.0;
        public static final double kDeployHomingStatorCurrentLimitAmps = 30.0;

        public static final double kHomingCurrentThresholdAmps = 15.0;
        public static final double kHomingMinRunTimeSeconds = 0.10;
        public static final double kHomingCurrentDebounceSeconds = 0.10;
        public static final double kHomingTimeoutSeconds = 10.0;

        public static final double kSysIdQuasistaticRampRateVoltsPerSecond = 0.25;
        public static final double kSysIdDynamicStepVolts = 2.0;
        public static final double kSysIdTimeoutSeconds = 10.0;

        public static final double kRollerMotorOutput = 0.95;
        // Preserves the previous right-follower direction now that the right roller is the lead.
        public static final double kRightRollerMotorOutputSign = -1.0;
        public static final double kRollerSupplyCurrentLimitAmps = 25.0;
        public static final double kRollerStatorCurrentLimitAmps = 40.0;

        public static final boolean kLeftRollerOpposesRight = true;
    }
}
