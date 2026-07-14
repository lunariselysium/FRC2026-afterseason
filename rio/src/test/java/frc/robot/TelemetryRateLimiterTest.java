package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class TelemetryRateLimiterTest {
    @Test
    void publishesInitialUpdateThenLimitsUpdatesToConfiguredPeriod() {
        TelemetryRateLimiter limiter = new TelemetryRateLimiter(0.1);

        assertTrue(limiter.shouldPublish(10.0));
        assertFalse(limiter.shouldPublish(10.099));
        assertTrue(limiter.shouldPublish(10.1));
    }
}
