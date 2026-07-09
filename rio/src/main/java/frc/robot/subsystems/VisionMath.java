// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

final class VisionMath {
    private VisionMath() {}

    static double getCameraYawDegreesToTarget(
        Pose2d robotPose,
        Transform3d robotToCamera,
        Pose3d targetPose
    ) {
        Pose3d cameraPose = new Pose3d(robotPose).plus(robotToCamera);
        Translation3d cameraToTarget =
            new Transform3d(cameraPose, targetPose).getTranslation();

        return Math.toDegrees(Math.atan2(cameraToTarget.getY(), cameraToTarget.getX()));
    }

    static double getCameraYawDegreesToFieldPoint(
        Pose2d robotPose,
        Transform3d robotToCamera,
        Translation2d fieldPoint
    ) {
        Pose3d cameraPose = new Pose3d(robotPose).plus(robotToCamera);
        Pose3d fieldPointPose = new Pose3d(
            fieldPoint.getX(),
            fieldPoint.getY(),
            cameraPose.getZ(),
            Rotation3d.kZero
        );

        return getCameraYawDegrees(cameraPose, fieldPointPose);
    }

    static double getCameraYawDegreesToFieldPointFromTag(
        Transform3d cameraToTag,
        Pose3d tagPose,
        Translation2d fieldPoint
    ) {
        return getCameraYawDegrees(
            getCameraToFieldPointFromTag(cameraToTag, tagPose, fieldPoint)
        );
    }

    static Translation3d getCameraToFieldPointFromTag(
        Transform3d cameraToTag,
        Pose3d tagPose,
        Translation2d fieldPoint
    ) {
        Pose3d fieldPointPose = new Pose3d(
            fieldPoint.getX(),
            fieldPoint.getY(),
            tagPose.getZ(),
            Rotation3d.kZero
        );
        Transform3d tagToFieldPoint = new Transform3d(tagPose, fieldPointPose);
        return cameraToTag.plus(tagToFieldPoint).getTranslation();
    }

    private static double getCameraYawDegrees(Pose3d cameraPose, Pose3d targetPose) {
        Translation3d cameraToTarget =
            new Transform3d(cameraPose, targetPose).getTranslation();

        return getCameraYawDegrees(cameraToTarget);
    }

    static double getCameraYawDegrees(Translation3d cameraToTarget) {
        return Math.toDegrees(Math.atan2(cameraToTarget.getY(), cameraToTarget.getX()));
    }

    static double getYawResidualDegrees(
        double observedYawDegrees,
        double expectedYawDegrees
    ) {
        return MathUtil.inputModulus(
            observedYawDegrees - expectedYawDegrees,
            -180.0,
            180.0
        );
    }
}
