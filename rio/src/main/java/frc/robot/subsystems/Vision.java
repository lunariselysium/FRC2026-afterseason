// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private static final AprilTagFieldLayout kFieldLayout =
        AprilTagFieldLayout.loadField(VisionConstants.kAprilTagField);

    private final CommandSwerveDrivetrain drivetrain;
    private final Turret turret;

    private final VisionCamera[] cameras;
    private final HubVisionObservation blueHubVision = new HubVisionObservation();
    private final HubVisionObservation redHubVision = new HubVisionObservation();

    public Vision(CommandSwerveDrivetrain drivetrain, Turret turret) {
        this.drivetrain = drivetrain;
        this.turret = turret;

        cameras = new VisionCamera[] {
            new VisionCamera(
                VisionConstants.kLeftFrameCameraName,
                () -> VisionConstants.kRobotToLeftFrameCamera,
                VisionConstants.kLeftFramePoseUpdatesEnabled,
                false
            ),
            new VisionCamera(
                VisionConstants.kRightFrameCameraName,
                () -> VisionConstants.kRobotToRightFrameCamera,
                VisionConstants.kRightFramePoseUpdatesEnabled,
                false
            ),
            new VisionCamera(
                VisionConstants.kBackFrameCameraName,
                () -> VisionConstants.kRobotToBackFrameCamera,
                VisionConstants.kBackFramePoseUpdatesEnabled,
                false
            ),
            new VisionCamera(
                VisionConstants.kTurretForwardCameraName,
                this::getRobotToTurretForwardCamera,
                VisionConstants.kTurretForwardPoseUpdatesEnabled,
                true
            ),
        };
    }

    public OptionalDouble getTurretForwardHubVisionCorrectionDegrees(Alliance alliance) {
        return getHubVisionObservation(alliance).getCorrectionDegreesIfFresh();
    }

    public void updateControlAndTelemetry() {
        for (VisionCamera camera : cameras) {
            processCamera(camera);
            publishCameraTelemetry(camera);
        }
        publishTurretForwardHubVisionTelemetry();
    }

    private Transform3d getRobotToTurretForwardCamera() {
        double turretYawRadians = Units.degreesToRadians(
            VisionConstants.kTurretHeadingToRobotYawSign * turret.getHeadingDegrees()
                + VisionConstants.kTurretForwardCameraZeroYawOffsetDegrees
        );

        Transform3d robotToTurretYaw =
            new Transform3d(new Translation3d(), new Rotation3d(0.0, 0.0, turretYawRadians));

        return VisionConstants.kRobotToTurretYawAxis
            .plus(robotToTurretYaw)
            .plus(VisionConstants.kTurretYawAxisToTurretForwardCamera);
    }

    private void processCamera(VisionCamera camera) {
        camera.connected = camera.photonCamera.isConnected();

        Transform3d robotToCamera = camera.robotToCameraSupplier.get();
        camera.poseEstimator.setRobotToCameraTransform(robotToCamera);

        for (PhotonPipelineResult result : camera.photonCamera.getAllUnreadResults()) {
            Pose2d robotPose = drivetrain.getState().Pose;
            updateTargetTelemetry(camera, result);
            updateTurretForwardHubVisionAssist(camera, result, robotPose, robotToCamera);

            Optional<EstimatedRobotPose> estimatedPose = camera.poseEstimator.update(result);
            if (estimatedPose.isEmpty()) {
                continue;
            }

            EstimatedRobotPose pose = estimatedPose.get();
            if (pose.timestampSeconds <= camera.lastValidEstimateTimestampSeconds) {
                continue;
            }

            double averageTagDistanceMeters = getAverageTagDistanceMeters(pose.targetsUsed);
            if (!shouldAcceptPose(pose.estimatedPose, pose.targetsUsed, averageTagDistanceMeters)) {
                camera.rejectedEstimateCount++;
                continue;
            }

            camera.lastValidEstimateTimestampSeconds = pose.timestampSeconds;
            camera.validEstimateCount++;

            if (!VisionConstants.kEnableVisionPoseFusion || !camera.poseUpdatesEnabled) {
                continue;
            }

            Matrix<N3, N1> stdDevs = getVisionStdDevs(
                pose.targetsUsed.size(),
                averageTagDistanceMeters,
                camera.dynamicRobotToCamera
            );

            drivetrain.addVisionMeasurement(
                pose.estimatedPose.toPose2d(),
                pose.timestampSeconds,
                stdDevs
            );

            camera.fusedPoseCount++;
        }
    }

    private void updateTurretForwardHubVisionAssist(
        VisionCamera camera,
        PhotonPipelineResult result,
        Pose2d robotPose,
        Transform3d robotToCamera
    ) {
        if (!camera.name.equals(VisionConstants.kTurretForwardCameraName)) {
            return;
        }

        updateHubVisionObservation(
            blueHubVision,
            result,
            ScoringConstants.kBlueHubTagIds,
            ScoringConstants.kBlueHubCenterMeters,
            robotPose,
            robotToCamera
        );
        updateHubVisionObservation(
            redHubVision,
            result,
            ScoringConstants.kRedHubTagIds,
            ScoringConstants.kRedHubCenterMeters,
            robotPose,
            robotToCamera
        );
    }

    private void updateHubVisionObservation(
        HubVisionObservation observation,
        PhotonPipelineResult result,
        int[] fiducialIds,
        Translation2d hubCenterMeters,
        Pose2d robotPose,
        Transform3d robotToCamera
    ) {
        double cameraToHubXTotalMeters = 0.0;
        double cameraToHubYTotalMeters = 0.0;
        double cameraToHubZTotalMeters = 0.0;
        int correctionCount = 0;
        int representativeTagId = -1;
        double representativeCorrectionMagnitudeDegrees = Double.POSITIVE_INFINITY;
        double expectedHubYawDegrees = VisionMath.getCameraYawDegreesToFieldPoint(
            robotPose,
            robotToCamera,
            hubCenterMeters
        );
        for (PhotonTrackedTarget target : result.getTargets()) {
            if (!containsFiducialId(fiducialIds, target.getFiducialId())) {
                continue;
            }

            Optional<Pose3d> tagPose = kFieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) {
                continue;
            }

            Translation3d cameraToHubCenter = VisionMath.getCameraToFieldPointFromTag(
                target.getBestCameraToTarget(),
                tagPose.get(),
                hubCenterMeters
            );
            cameraToHubXTotalMeters += cameraToHubCenter.getX();
            cameraToHubYTotalMeters += cameraToHubCenter.getY();
            cameraToHubZTotalMeters += cameraToHubCenter.getZ();
            correctionCount++;

            double observedHubYawDegrees = VisionMath.getCameraYawDegrees(cameraToHubCenter);
            double tagCorrectionDegrees = VisionMath.getYawResidualDegrees(
                observedHubYawDegrees,
                expectedHubYawDegrees
            );

            double correctionMagnitudeDegrees = Math.abs(tagCorrectionDegrees);
            if (correctionMagnitudeDegrees < representativeCorrectionMagnitudeDegrees) {
                representativeCorrectionMagnitudeDegrees = correctionMagnitudeDegrees;
                representativeTagId = target.getFiducialId();
            }
        }

        if (correctionCount == 0) {
            observation.clear();
            return;
        }

        Translation3d averageCameraToHubCenter = new Translation3d(
            cameraToHubXTotalMeters / correctionCount,
            cameraToHubYTotalMeters / correctionCount,
            cameraToHubZTotalMeters / correctionCount
        );
        double averagedHubYawDegrees =
            VisionMath.getCameraYawDegrees(averageCameraToHubCenter);
        observation.update(
            VisionMath.getYawResidualDegrees(averagedHubYawDegrees, expectedHubYawDegrees),
            representativeTagId,
            correctionCount
        );
    }

    private boolean containsFiducialId(int[] fiducialIds, int fiducialId) {
        for (int expectedFiducialId : fiducialIds) {
            if (expectedFiducialId == fiducialId) {
                return true;
            }
        }

        return false;
    }

    private void updateTargetTelemetry(VisionCamera camera, PhotonPipelineResult result) {
        camera.hasTargets = result.hasTargets();
        camera.lastTargetCount = result.getTargets().size();

        if (!result.hasTargets()) {
            camera.bestTargetYawDegrees = 0.0;
            return;
        }

        PhotonTrackedTarget bestTarget = result.getBestTarget();
        camera.bestTargetYawDegrees = bestTarget.getYaw();
    }

    private boolean shouldAcceptPose(
        Pose3d estimatedPose,
        List<PhotonTrackedTarget> targets,
        double averageTagDistanceMeters
    ) {
        if (targets.isEmpty()) {
            return false;
        }

        if (Math.abs(estimatedPose.getZ()) > VisionConstants.kMaxAcceptedPoseZErrorMeters) {
            return false;
        }

        if (estimatedPose.getX() < -VisionConstants.kAcceptedFieldBoundaryMarginMeters
            || estimatedPose.getX() > kFieldLayout.getFieldLength() + VisionConstants.kAcceptedFieldBoundaryMarginMeters
            || estimatedPose.getY() < -VisionConstants.kAcceptedFieldBoundaryMarginMeters
            || estimatedPose.getY() > kFieldLayout.getFieldWidth() + VisionConstants.kAcceptedFieldBoundaryMarginMeters) {
            return false;
        }

        if (averageTagDistanceMeters > VisionConstants.kMaxAverageTagDistanceMeters) {
            return false;
        }

        if (targets.size() == 1) {
            double ambiguity = targets.get(0).getPoseAmbiguity();
            return ambiguity >= 0.0 && ambiguity <= VisionConstants.kMaxSingleTagAmbiguity;
        }

        return true;
    }

    private Matrix<N3, N1> getVisionStdDevs(
        int tagCount,
        double averageTagDistanceMeters,
        boolean dynamicRobotToCamera
    ) {
        double tagCountScale = tagCount <= 1
            ? VisionConstants.kSingleTagStdDevMultiplier
            : 1.0 / Math.sqrt(tagCount);
        double distanceScale = Math.max(1.0, averageTagDistanceMeters * averageTagDistanceMeters / 4.0);
        double cameraScale = dynamicRobotToCamera
            ? VisionConstants.kTurretCameraStdDevMultiplier
            : 1.0;

        double translationStdDevMeters = VisionConstants.kBaseTranslationStdDevMeters
            * tagCountScale
            * distanceScale
            * cameraScale
            * 2.0;
        double rotationStdDevRadians = VisionConstants.kBaseRotationStdDevRadians
            * tagCountScale
            * distanceScale
            * cameraScale
            * 2.0;

        return VecBuilder.fill(
            translationStdDevMeters,
            translationStdDevMeters,
            rotationStdDevRadians
        );
    }

    private double getAverageTagDistanceMeters(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) {
            return Double.POSITIVE_INFINITY;
        }

        double totalDistanceMeters = 0.0;
        for (PhotonTrackedTarget target : targets) {
            totalDistanceMeters += target.getBestCameraToTarget().getTranslation().getNorm();
        }

        return totalDistanceMeters / targets.size();
    }

    private void publishCameraTelemetry(VisionCamera camera) {
        String dashboardKey = "Vision/" + camera.name;

        SmartDashboard.putBoolean(dashboardKey + "/Connected", camera.connected);
        SmartDashboard.putBoolean(dashboardKey + "/HasTargets", camera.hasTargets);
        SmartDashboard.putNumber(dashboardKey + "/TargetCount", camera.lastTargetCount);
        SmartDashboard.putNumber(dashboardKey + "/BestTargetYawDegrees", camera.bestTargetYawDegrees);
        SmartDashboard.putNumber(dashboardKey + "/ValidEstimateCount", camera.validEstimateCount);
        SmartDashboard.putNumber(dashboardKey + "/RejectedEstimateCount", camera.rejectedEstimateCount);
        SmartDashboard.putNumber(dashboardKey + "/FusedPoseCount", camera.fusedPoseCount);
    }

    private HubVisionObservation getHubVisionObservation(Alliance alliance) {
        return alliance == Alliance.Red ? redHubVision : blueHubVision;
    }

    private void publishTurretForwardHubVisionTelemetry() {
        publishHubVisionTelemetry("Vision/TurretForwardHubAssist/Blue", blueHubVision);
        publishHubVisionTelemetry("Vision/TurretForwardHubAssist/Red", redHubVision);
    }

    private void publishHubVisionTelemetry(
        String dashboardKey,
        HubVisionObservation observation
    ) {
        SmartDashboard.putBoolean(
            dashboardKey + "/HasCorrection",
            observation.hasFreshCorrection()
        );
        SmartDashboard.putNumber(dashboardKey + "/CorrectionDegrees", observation.correctionDegrees);
        SmartDashboard.putNumber(dashboardKey + "/RepresentativeTagId", observation.representativeTagId);
        SmartDashboard.putNumber(dashboardKey + "/TargetCount", observation.targetCount);
    }

    private static final class VisionCamera {
        private final String name;
        private final PhotonCamera photonCamera;
        private final PhotonPoseEstimator poseEstimator;
        private final Supplier<Transform3d> robotToCameraSupplier;
        private final boolean poseUpdatesEnabled;
        private final boolean dynamicRobotToCamera;

        private boolean connected;
        private boolean hasTargets;
        private int lastTargetCount;
        private int validEstimateCount;
        private int rejectedEstimateCount;
        private int fusedPoseCount;
        private double bestTargetYawDegrees;
        private double lastValidEstimateTimestampSeconds = -1.0;

        private VisionCamera(
            String name,
            Supplier<Transform3d> robotToCameraSupplier,
            boolean poseUpdatesEnabled,
            boolean dynamicRobotToCamera
        ) {
            this.name = name;
            this.robotToCameraSupplier = robotToCameraSupplier;
            this.poseUpdatesEnabled = poseUpdatesEnabled;
            this.dynamicRobotToCamera = dynamicRobotToCamera;

            photonCamera = new PhotonCamera(name);
            poseEstimator = new PhotonPoseEstimator(
                kFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCameraSupplier.get()
            );
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }
    }

    private static final class HubVisionObservation {
        private boolean hasCorrection;
        private double correctionDegrees;
        private int representativeTagId = -1;
        private int targetCount;
        private double timestampSeconds = -1.0;

        private void update(
            double correctionDegrees,
            int representativeTagId,
            int targetCount
        ) {
            hasCorrection = true;
            this.correctionDegrees = correctionDegrees;
            this.representativeTagId = representativeTagId;
            this.targetCount = targetCount;
            timestampSeconds = Timer.getFPGATimestamp();
        }

        private void clear() {
            hasCorrection = false;
            correctionDegrees = 0.0;
            representativeTagId = -1;
            targetCount = 0;
            timestampSeconds = -1.0;
        }

        private OptionalDouble getCorrectionDegreesIfFresh() {
            if (!hasFreshCorrection()) {
                return OptionalDouble.empty();
            }

            return OptionalDouble.of(correctionDegrees);
        }

        private boolean hasFreshCorrection() {
            return hasCorrection
                && Timer.getFPGATimestamp() - timestampSeconds
                    <= ScoringConstants.kHubVisionAssistStaleSeconds;
        }
    }
}
