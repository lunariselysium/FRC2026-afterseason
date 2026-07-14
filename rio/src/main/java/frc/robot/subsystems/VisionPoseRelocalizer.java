// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

final class VisionPoseRelocalizer {
    private final int requiredSampleCount;
    private final double maxTranslationSpreadMeters;
    private final double maxRotationSpreadRadians;
    private final List<Pose2d> samples = new ArrayList<>();

    private Optional<Pose2d> lockedPose = Optional.empty();

    VisionPoseRelocalizer(
        int requiredSampleCount,
        double maxTranslationSpreadMeters,
        double maxRotationSpreadRadians
    ) {
        if (requiredSampleCount <= 0) {
            throw new IllegalArgumentException("requiredSampleCount must be positive");
        }
        if (maxTranslationSpreadMeters < 0.0 || maxRotationSpreadRadians < 0.0) {
            throw new IllegalArgumentException("pose spread tolerances cannot be negative");
        }

        this.requiredSampleCount = requiredSampleCount;
        this.maxTranslationSpreadMeters = maxTranslationSpreadMeters;
        this.maxRotationSpreadRadians = maxRotationSpreadRadians;
    }

    Optional<Pose2d> addSample(Pose2d sample) {
        if (lockedPose.isPresent()) {
            return lockedPose;
        }

        if (!isConsistentWithCurrentSamples(sample)) {
            samples.clear();
        }
        samples.add(sample);

        if (samples.size() < requiredSampleCount) {
            return Optional.empty();
        }

        lockedPose = Optional.of(averageSamples());
        return lockedPose;
    }

    void reset() {
        samples.clear();
        lockedPose = Optional.empty();
    }

    int getSampleCount() {
        return samples.size();
    }

    private boolean isConsistentWithCurrentSamples(Pose2d sample) {
        if (samples.isEmpty()) {
            return true;
        }

        Pose2d averagePose = averageSamples();
        if (averagePose.getTranslation().getDistance(sample.getTranslation())
            > maxTranslationSpreadMeters) {
            return false;
        }

        double rotationDifferenceRadians = Math.abs(
            MathUtil.angleModulus(
                sample.getRotation().minus(averagePose.getRotation()).getRadians()
            )
        );
        return rotationDifferenceRadians <= maxRotationSpreadRadians;
    }

    private Pose2d averageSamples() {
        double xTotalMeters = 0.0;
        double yTotalMeters = 0.0;
        double rotationCosineTotal = 0.0;
        double rotationSineTotal = 0.0;

        for (Pose2d sample : samples) {
            xTotalMeters += sample.getX();
            yTotalMeters += sample.getY();
            rotationCosineTotal += sample.getRotation().getCos();
            rotationSineTotal += sample.getRotation().getSin();
        }

        double sampleCount = samples.size();
        return new Pose2d(
            xTotalMeters / sampleCount,
            yTotalMeters / sampleCount,
            new Rotation2d(
                rotationCosineTotal / sampleCount,
                rotationSineTotal / sampleCount
            )
        );
    }
}
