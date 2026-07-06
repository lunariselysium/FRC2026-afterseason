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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
    private final HubYawObservation blueHubYaw = new HubYawObservation();
    private final HubYawObservation redHubYaw = new HubYawObservation();

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

    public OptionalDouble getTurretForwardHubYawDegrees(Alliance alliance) {
        return getHubYawObservation(alliance).getYawDegreesIfFresh();
    }

    public void updateControlAndTelemetry() {
        for (VisionCamera camera : cameras) {
            processCamera(camera);
            publishCameraTelemetry(camera);
        }
        publishTurretForwardHubYawTelemetry();
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
            updateTargetTelemetry(camera, result);
            updateTurretForwardHubYaw(camera, result);

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

    private void updateTurretForwardHubYaw(VisionCamera camera, PhotonPipelineResult result) {
        if (!camera.name.equals(VisionConstants.kTurretForwardCameraName)) {
            return;
        }

        updateHubYawObservation(blueHubYaw, result, ScoringConstants.kBlueHubTagIds);
        updateHubYawObservation(redHubYaw, result, ScoringConstants.kRedHubTagIds);
    }

    private void updateHubYawObservation(
        HubYawObservation observation,
        PhotonPipelineResult result,
        int[] fiducialIds
    ) {
        Optional<PhotonTrackedTarget> target = getTargetByFiducialIds(result, fiducialIds);
        target.ifPresent(observation::update);
    }

    private Optional<PhotonTrackedTarget> getTargetByFiducialIds(
        PhotonPipelineResult result,
        int[] fiducialIds
    ) {
        PhotonTrackedTarget bestTarget = null;
        for (PhotonTrackedTarget target : result.getTargets()) {
            if (!containsFiducialId(fiducialIds, target.getFiducialId())) {
                continue;
            }

            if (bestTarget == null
                || Math.abs(target.getYaw()) < Math.abs(bestTarget.getYaw())) {
                bestTarget = target;
            }
        }

        return Optional.ofNullable(bestTarget);
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
            * cameraScale;
        double rotationStdDevRadians = VisionConstants.kBaseRotationStdDevRadians
            * tagCountScale
            * distanceScale
            * cameraScale;

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

    private HubYawObservation getHubYawObservation(Alliance alliance) {
        return alliance == Alliance.Red ? redHubYaw : blueHubYaw;
    }

    private void publishTurretForwardHubYawTelemetry() {
        publishHubYawTelemetry("Vision/TurretForwardHubYaw/Blue", blueHubYaw);
        publishHubYawTelemetry("Vision/TurretForwardHubYaw/Red", redHubYaw);
    }

    private void publishHubYawTelemetry(String dashboardKey, HubYawObservation observation) {
        SmartDashboard.putBoolean(dashboardKey + "/HasTarget", observation.hasFreshTarget());
        SmartDashboard.putNumber(dashboardKey + "/YawDegrees", observation.yawDegrees);
        SmartDashboard.putNumber(dashboardKey + "/TagId", observation.tagId);
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

    private static final class HubYawObservation {
        private boolean hasTarget;
        private double yawDegrees;
        private int tagId = -1;
        private double timestampSeconds = -1.0;

        private void update(PhotonTrackedTarget target) {
            hasTarget = true;
            yawDegrees = target.getYaw();
            tagId = target.getFiducialId();
            timestampSeconds = Timer.getFPGATimestamp();
        }

        private OptionalDouble getYawDegreesIfFresh() {
            if (!hasFreshTarget()) {
                return OptionalDouble.empty();
            }

            return OptionalDouble.of(yawDegrees);
        }

        private boolean hasFreshTarget() {
            return hasTarget
                && Timer.getFPGATimestamp() - timestampSeconds
                    <= ScoringConstants.kHubVisualYawStaleSeconds;
        }
    }
}
