// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class VisionPoseRelocalizerTest {
    private static final double kTolerance = 1.0e-9;

    @Test
    void locksAfterRequiredConsistentSamplesAndAveragesThem() {
        VisionPoseRelocalizer relocalizer = new VisionPoseRelocalizer(
            3,
            0.25,
            Math.toRadians(5.0)
        );

        assertTrue(relocalizer.addSample(new Pose2d(5.00, 2.00, Rotation2d.fromDegrees(179.0))).isEmpty());
        assertTrue(relocalizer.addSample(new Pose2d(5.10, 1.90, Rotation2d.fromDegrees(-179.0))).isEmpty());
        Optional<Pose2d> lockedPose = relocalizer.addSample(
            new Pose2d(4.90, 2.10, Rotation2d.fromDegrees(180.0))
        );

        assertTrue(lockedPose.isPresent());
        assertEquals(5.0, lockedPose.get().getX(), kTolerance);
        assertEquals(2.0, lockedPose.get().getY(), kTolerance);
        assertEquals(
            180.0,
            Math.abs(lockedPose.get().getRotation().getDegrees()),
            kTolerance
        );
    }

    @Test
    void discardsOldClusterWhenAnInconsistentSampleArrives() {
        VisionPoseRelocalizer relocalizer = new VisionPoseRelocalizer(
            3,
            0.20,
            Math.toRadians(5.0)
        );

        relocalizer.addSample(new Pose2d(1.00, 1.00, Rotation2d.kZero));
        relocalizer.addSample(new Pose2d(1.05, 1.00, Rotation2d.fromDegrees(1.0)));
        assertTrue(
            relocalizer.addSample(new Pose2d(4.00, 4.00, Rotation2d.fromDegrees(90.0))).isEmpty()
        );
        assertEquals(1, relocalizer.getSampleCount());

        assertTrue(
            relocalizer.addSample(new Pose2d(4.05, 4.00, Rotation2d.fromDegrees(91.0))).isEmpty()
        );
        assertTrue(
            relocalizer.addSample(new Pose2d(3.95, 4.00, Rotation2d.fromDegrees(89.0))).isPresent()
        );
    }
}
