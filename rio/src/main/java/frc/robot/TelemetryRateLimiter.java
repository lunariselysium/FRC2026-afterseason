package frc.robot;

public final class TelemetryRateLimiter {
    private static final double kRobotLoopPeriodSeconds = 0.02;
    private static final double kRobotTelemetryPeriodSeconds = 0.1;
    private static final int kRobotTelemetryPhaseCount = 5;
    private static final double kTimestampToleranceSeconds = 1.0e-9;

    private final double updatePeriodSeconds;
    private final double initialDelaySeconds;
    private double lastPublishedTimestampSeconds = Double.NEGATIVE_INFINITY;
    private boolean initialized;

    public TelemetryRateLimiter(double updatePeriodSeconds) {
        this(updatePeriodSeconds, 0.0);
    }

    private TelemetryRateLimiter(double updatePeriodSeconds, double initialDelaySeconds) {
        this.updatePeriodSeconds = updatePeriodSeconds;
        this.initialDelaySeconds = initialDelaySeconds;
    }

    public static TelemetryRateLimiter forRobotTelemetry() {
        return new TelemetryRateLimiter(kRobotTelemetryPeriodSeconds);
    }

    /** Creates a 10 Hz limiter offset by the requested 20 ms robot-loop phase. */
    public static TelemetryRateLimiter forRobotTelemetryPhase(int phase) {
        if (phase < 0 || phase >= kRobotTelemetryPhaseCount) {
            throw new IllegalArgumentException("Telemetry phase must be between 0 and 4");
        }

        return new TelemetryRateLimiter(
            kRobotTelemetryPeriodSeconds,
            phase * kRobotLoopPeriodSeconds
        );
    }

    public boolean shouldPublish(double timestampSeconds) {
        if (!initialized) {
            lastPublishedTimestampSeconds = timestampSeconds
                - updatePeriodSeconds
                + initialDelaySeconds;
            initialized = true;
        }

        if (timestampSeconds - lastPublishedTimestampSeconds
            < updatePeriodSeconds - kTimestampToleranceSeconds) {
            return false;
        }

        lastPublishedTimestampSeconds = timestampSeconds;
        return true;
    }
}
