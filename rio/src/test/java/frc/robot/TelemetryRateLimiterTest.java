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

    @Test
    void robotTelemetryPublishesAtTenHertzDuringTwentyMillisecondLoops() {
        TelemetryRateLimiter limiter = TelemetryRateLimiter.forRobotTelemetry();

        assertTrue(limiter.shouldPublish(20.00));
        assertFalse(limiter.shouldPublish(20.02));
        assertFalse(limiter.shouldPublish(20.04));
        assertFalse(limiter.shouldPublish(20.06));
        assertFalse(limiter.shouldPublish(20.08));
        assertTrue(limiter.shouldPublish(20.10));
    }

    @Test
    void phasedRobotTelemetryStaggersItsFirstPublication() {
        TelemetryRateLimiter limiter = TelemetryRateLimiter.forRobotTelemetryPhase(2);

        assertFalse(limiter.shouldPublish(30.00));
        assertFalse(limiter.shouldPublish(30.02));
        assertTrue(limiter.shouldPublish(30.04));
        assertFalse(limiter.shouldPublish(30.12));
        assertTrue(limiter.shouldPublish(30.14));
    }
}
