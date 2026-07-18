// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class FeederJamRecoveryControllerTest {
    @Test
    void startsRecoveryAfterOneMotorStaysJammedForQualificationTime() {
        FeederJamRecoveryController controller = new FeederJamRecoveryController(0.20, 0.20);

        assertEquals(
            FeederJamRecoveryController.OutputMode.NOMINAL,
            controller.update(1.00, true, true, false, false, false)
        );
        assertEquals(
            FeederJamRecoveryController.OutputMode.NOMINAL,
            controller.update(1.19, true, true, false, false, false)
        );
        assertEquals(
            FeederJamRecoveryController.OutputMode.REVERSE,
            controller.update(1.20, true, true, false, false, false)
        );
    }

    @Test
    void requiresOneMotorToRemainContinuouslyJammed() {
        FeederJamRecoveryController controller = new FeederJamRecoveryController(0.20, 0.20);

        controller.update(1.00, true, true, false, false, false);
        controller.update(1.15, true, false, false, false, false);

        assertEquals(
            FeederJamRecoveryController.OutputMode.NOMINAL,
            controller.update(1.25, true, true, false, false, false)
        );
        assertEquals(
            FeederJamRecoveryController.OutputMode.NOMINAL,
            controller.update(1.44, true, true, false, false, false)
        );
        assertEquals(
            FeederJamRecoveryController.OutputMode.REVERSE,
            controller.update(1.45, true, true, false, false, false)
        );
    }

    @Test
    void reversesForConfiguredRecoveryTimeThenResumesNominalOutput() {
        FeederJamRecoveryController controller = new FeederJamRecoveryController(0.20, 0.20);

        controller.update(1.00, true, false, true, false, false);
        assertEquals(
            FeederJamRecoveryController.OutputMode.REVERSE,
            controller.update(1.20, true, false, true, false, false)
        );
        assertEquals(
            FeederJamRecoveryController.OutputMode.REVERSE,
            controller.update(1.39, true, false, true, false, false)
        );
        assertEquals(
            FeederJamRecoveryController.OutputMode.NOMINAL,
            controller.update(1.40, true, false, true, false, false)
        );
    }

    @Test
    void cancelsRecoveryWhenNoForwardFeedIsRequested() {
        FeederJamRecoveryController controller = new FeederJamRecoveryController(0.20, 0.20);

        controller.update(1.00, true, false, false, true, false);
        controller.update(1.20, true, false, false, true, false);

        assertEquals(
            FeederJamRecoveryController.OutputMode.NOMINAL,
            controller.update(1.21, false, false, false, true, false)
        );
        assertEquals(false, controller.isRecoveryActive());
    }

    @Test
    void anyOfTheFourFeederMotorsCanTriggerRecovery() {
        for (int jammedMotorIndex = 0; jammedMotorIndex < 4; jammedMotorIndex++) {
            FeederJamRecoveryController controller =
                new FeederJamRecoveryController(0.20, 0.20);

            updateWithOnlyOneMotorJammed(controller, 1.00, jammedMotorIndex);

            assertEquals(
                FeederJamRecoveryController.OutputMode.REVERSE,
                updateWithOnlyOneMotorJammed(controller, 1.20, jammedMotorIndex),
                "Motor index " + jammedMotorIndex + " did not trigger recovery"
            );
        }
    }

    private FeederJamRecoveryController.OutputMode updateWithOnlyOneMotorJammed(
        FeederJamRecoveryController controller,
        double timestampSeconds,
        int jammedMotorIndex
    ) {
        return controller.update(
            timestampSeconds,
            true,
            jammedMotorIndex == 0,
            jammedMotorIndex == 1,
            jammedMotorIndex == 2,
            jammedMotorIndex == 3
        );
    }
}
