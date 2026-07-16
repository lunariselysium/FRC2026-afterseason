// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

final class TurretHeadingMath {
    private static final double kEpsilon = 1.0e-9;

    private TurretHeadingMath() {}

    record EncoderUnwrapResult(
        double encoderRotationsFromForward,
        double motorGuidanceErrorDegrees,
        boolean usedMotorGuidance
    ) {}

    static double chooseNearestEquivalentInWindow(
        double requestedHeadingDegrees,
        double referenceHeadingDegrees,
        double minHeadingDegrees,
        double maxHeadingDegrees,
        double wrapDegrees
    ) {
        if (wrapDegrees <= 0.0) {
            return clampHeadingToTravelWindow(
                requestedHeadingDegrees,
                minHeadingDegrees,
                maxHeadingDegrees
            );
        }

        double minWraps = Math.ceil((minHeadingDegrees - requestedHeadingDegrees) / wrapDegrees);
        double maxWraps = Math.floor((maxHeadingDegrees - requestedHeadingDegrees) / wrapDegrees);

        if (minWraps > maxWraps) {
            return clampHeadingToTravelWindow(
                requestedHeadingDegrees,
                minHeadingDegrees,
                maxHeadingDegrees
            );
        }

        double preferredWraps = Math.round(
            (referenceHeadingDegrees - requestedHeadingDegrees) / wrapDegrees
        );
        double bestWraps = clamp(preferredWraps, minWraps, maxWraps);
        double bestHeadingDegrees = requestedHeadingDegrees + bestWraps * wrapDegrees;

        double lowerNeighborWraps = Math.max(minWraps, bestWraps - 1.0);
        double upperNeighborWraps = Math.min(maxWraps, bestWraps + 1.0);
        for (double wraps = lowerNeighborWraps; wraps <= upperNeighborWraps; wraps += 1.0) {
            double candidateHeadingDegrees = requestedHeadingDegrees + wraps * wrapDegrees;
            if (isBetterCandidate(
                candidateHeadingDegrees,
                bestHeadingDegrees,
                requestedHeadingDegrees,
                referenceHeadingDegrees
            )) {
                bestHeadingDegrees = candidateHeadingDegrees;
            }
        }

        return clampHeadingToTravelWindow(
            bestHeadingDegrees,
            minHeadingDegrees,
            maxHeadingDegrees
        );
    }

    static double clampHeadingToTravelWindow(
        double headingDegrees,
        double minHeadingDegrees,
        double maxHeadingDegrees
    ) {
        return clamp(headingDegrees, minHeadingDegrees, maxHeadingDegrees);
    }

    static boolean isWithinTolerance(
        double targetHeadingDegrees,
        double measuredHeadingDegrees,
        double toleranceDegrees
    ) {
        return Math.abs(targetHeadingDegrees - measuredHeadingDegrees) <= toleranceDegrees;
    }

    static double encoderRotationsToDegrees(double encoderRotations) {
        return encoderRotations * 360.0;
    }

    static double initializeEncoderRotationsFromKnownHeading(
        double rawEncoderRotations,
        double forwardEncoderOffsetDegrees,
        double knownStartupHeadingDegrees,
        double headingDegreesPerEncoderRotation
    ) {
        return chooseNearestEquivalentEncoderRotationsFromForward(
            rawEncoderRotations - forwardEncoderOffsetDegrees / 360.0,
            knownStartupHeadingDegrees,
            headingDegreesPerEncoderRotation
        );
    }

    static double chooseNearestEquivalentEncoderRotationsFromForward(
        double rawEncoderRotationsFromForward,
        double referenceHeadingDegrees,
        double headingDegreesPerEncoderRotation
    ) {
        if (Math.abs(headingDegreesPerEncoderRotation) <= kEpsilon) {
            return rawEncoderRotationsFromForward;
        }

        double referenceEncoderRotations = referenceHeadingDegrees
            / headingDegreesPerEncoderRotation;
        return rawEncoderRotationsFromForward
            + Math.round(referenceEncoderRotations - rawEncoderRotationsFromForward);
    }

    static double shiftEncoderRotationTowardHeadingDirection(
        double unwrappedEncoderRotations,
        double headingDirection,
        double headingDegreesPerEncoderRotation
    ) {
        if (Math.abs(headingDirection) <= kEpsilon
            || Math.abs(headingDegreesPerEncoderRotation) <= kEpsilon) {
            return unwrappedEncoderRotations;
        }

        return unwrappedEncoderRotations
            + Math.copySign(1.0, headingDirection * headingDegreesPerEncoderRotation);
    }

    static EncoderUnwrapResult chooseEncoderUnwrap(
        double rawEncoderRotationsFromForward,
        double continuityEncoderRotationsFromForward,
        double motorPredictedHeadingDegrees,
        double headingDegreesPerEncoderRotation
    ) {
        if (Math.abs(headingDegreesPerEncoderRotation) <= kEpsilon) {
            return new EncoderUnwrapResult(
                continuityEncoderRotationsFromForward,
                0.0,
                false
            );
        }

        double motorGuidedEncoderRotations =
            chooseNearestEquivalentEncoderRotationsFromForward(
                rawEncoderRotationsFromForward,
                motorPredictedHeadingDegrees,
                headingDegreesPerEncoderRotation
            );
        double motorGuidanceErrorDegrees = Math.abs(
            motorGuidedEncoderRotations * headingDegreesPerEncoderRotation
                - motorPredictedHeadingDegrees
        );
        double maximumUnambiguousGuidanceErrorDegrees =
            Math.abs(headingDegreesPerEncoderRotation) * 0.25;
        boolean usedMotorGuidance =
            motorGuidanceErrorDegrees <= maximumUnambiguousGuidanceErrorDegrees
                && Math.abs(
                    motorGuidedEncoderRotations - continuityEncoderRotationsFromForward
                ) > kEpsilon;

        return new EncoderUnwrapResult(
            usedMotorGuidance
                ? motorGuidedEncoderRotations
                : continuityEncoderRotationsFromForward,
            motorGuidanceErrorDegrees,
            usedMotorGuidance
        );
    }

    private static boolean isBetterCandidate(
        double candidateHeadingDegrees,
        double bestHeadingDegrees,
        double requestedHeadingDegrees,
        double referenceHeadingDegrees
    ) {
        double candidateDistance = Math.abs(candidateHeadingDegrees - referenceHeadingDegrees);
        double bestDistance = Math.abs(bestHeadingDegrees - referenceHeadingDegrees);

        if (candidateDistance < bestDistance - kEpsilon) {
            return true;
        }

        if (Math.abs(candidateDistance - bestDistance) > kEpsilon) {
            return false;
        }

        return Math.abs(candidateHeadingDegrees - requestedHeadingDegrees)
            < Math.abs(bestHeadingDegrees - requestedHeadingDegrees);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
