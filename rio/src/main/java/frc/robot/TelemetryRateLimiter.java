package frc.robot;

final class TelemetryRateLimiter {
    private static final double kTimestampToleranceSeconds = 1.0e-9;

    private final double updatePeriodSeconds;
    private double lastPublishedTimestampSeconds = Double.NEGATIVE_INFINITY;

    TelemetryRateLimiter(double updatePeriodSeconds) {
        this.updatePeriodSeconds = updatePeriodSeconds;
    }

    boolean shouldPublish(double timestampSeconds) {
        if (timestampSeconds - lastPublishedTimestampSeconds
            < updatePeriodSeconds - kTimestampToleranceSeconds) {
            return false;
        }

        lastPublishedTimestampSeconds = timestampSeconds;
        return true;
    }
}
