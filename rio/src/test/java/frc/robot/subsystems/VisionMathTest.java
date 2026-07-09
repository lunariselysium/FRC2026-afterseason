// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

class VisionMathTest {
    private static final double kTolerance = 1.0e-9;

    @Test
    void computesCameraYawToKnownFieldTarget() {
        double yawDegrees = VisionMath.getCameraYawDegreesToTarget(
            new Pose2d(1.0, 2.0, Rotation2d.kZero),
            new Transform3d(),
            new Pose3d(6.0, 5.0, 0.0, Rotation3d.kZero)
        );

        assertEquals(Math.toDegrees(Math.atan2(3.0, 5.0)), yawDegrees, kTolerance);
    }

    @Test
    void computesCameraYawWithRobotToCameraOffset() {
        double yawDegrees = VisionMath.getCameraYawDegreesToTarget(
            new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90.0)),
            new Transform3d(new Translation3d(1.0, 0.0, 0.0), Rotation3d.kZero),
            new Pose3d(1.0, 6.0, 0.0, Rotation3d.kZero)
        );

        assertEquals(0.0, yawDegrees, kTolerance);
    }

    @Test
    void wrapsYawResidualAcrossHalfTurnBoundary() {
        assertEquals(
            2.0,
            VisionMath.getYawResidualDegrees(-179.0, 179.0),
            kTolerance
        );
    }

    @Test
    void projectsObservedTagToFieldPointBeforeComputingYaw() {
        Translation3d cameraToFieldPoint = VisionMath.getCameraToFieldPointFromTag(
            new Transform3d(new Translation3d(5.0, 0.0, 0.0), Rotation3d.kZero),
            new Pose3d(5.0, 0.0, 0.0, Rotation3d.kZero),
            new Translation2d(5.0, 2.0)
        );
        double yawDegrees = VisionMath.getCameraYawDegreesToFieldPointFromTag(
            new Transform3d(new Translation3d(5.0, 0.0, 0.0), Rotation3d.kZero),
            new Pose3d(5.0, 0.0, 0.0, Rotation3d.kZero),
            new Translation2d(5.0, 2.0)
        );

        assertEquals(5.0, cameraToFieldPoint.getX(), kTolerance);
        assertEquals(2.0, cameraToFieldPoint.getY(), kTolerance);
        assertEquals(Math.toDegrees(Math.atan2(2.0, 5.0)), yawDegrees, kTolerance);
    }
}
