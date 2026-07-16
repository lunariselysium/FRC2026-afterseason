package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.IntakeConstants;

class IntakeHomingPlanTest {
    @Test
    void homesPastTheDeployedPositionBeforeCalibratingTheDeployedPosition() {
        IntakeHomingPlan homingPlan = IntakeHomingPlan.toDeployedPosition(
            IntakeConstants.kDeployedSetpointMotorRotations,
            IntakeConstants.kDeployedHardstopCaptureWindowMotorRotations
        );

        assertEquals(
            IntakeConstants.kDeployedSetpointMotorRotations
                + IntakeConstants.kDeployedHardstopCaptureWindowMotorRotations,
            homingPlan.motionTargetPositionMotorRotations()
        );
        assertEquals(
            IntakeConstants.kDeployedSetpointMotorRotations,
            homingPlan.calibratedPositionMotorRotations()
        );
        assertTrue(homingPlan.targetDeployed());
        assertTrue(homingPlan.usesNormalDeployMotionProfile());
    }

    @Test
    void homesBackwardAtStowWhenRetractIsRequestedBeforeCalibration() {
        IntakeHomingPlan homingPlan = IntakeHomingPlan.toStowedPosition(
            IntakeConstants.kDeployedSetpointMotorRotations
        );

        assertEquals(
            -IntakeConstants.kDeployedSetpointMotorRotations,
            homingPlan.motionTargetPositionMotorRotations()
        );
        assertEquals(0.0, homingPlan.calibratedPositionMotorRotations());
        assertFalse(homingPlan.targetDeployed());
        assertTrue(homingPlan.usesNormalDeployMotionProfile());
    }
}
