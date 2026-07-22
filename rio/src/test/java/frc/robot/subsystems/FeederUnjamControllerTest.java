// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class FeederUnjamControllerTest {
    @Test
    void acceptsOneRequestAndRunsForTheConfiguredDuration() {
        FeederUnjamController controller = new FeederUnjamController(0.20);

        assertTrue(controller.request(1.00));
        assertTrue(controller.update(1.19));
        assertFalse(controller.update(1.20));
    }

    @Test
    void rejectsAnotherControllersRequestWhileTheProcedureIsActive() {
        FeederUnjamController controller = new FeederUnjamController(0.20);

        assertTrue(controller.request(1.00));
        assertFalse(controller.request(1.00));
        assertTrue(controller.update(1.19));
    }

    @Test
    void acceptsANewPressAfterThePreviousProcedureFinishes() {
        FeederUnjamController controller = new FeederUnjamController(0.20);

        assertTrue(controller.request(1.00));
        assertTrue(controller.request(1.20));
        assertTrue(controller.update(1.39));
        assertFalse(controller.update(1.40));
    }

    @Test
    void cancelEndsTheProcedureImmediately() {
        FeederUnjamController controller = new FeederUnjamController(0.20);

        controller.request(1.00);
        controller.cancel();

        assertFalse(controller.isActive());
    }
}
