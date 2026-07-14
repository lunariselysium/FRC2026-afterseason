package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

class DriveCurrentLimitControllerTest {
    @Test
    void appliesShootingLimitOnceAndRestoresNormalLimitOnce() {
        List<Double> appliedLimits = new ArrayList<>();
        DriveCurrentLimitController currentLimitController = new DriveCurrentLimitController(
            70.0,
            40.0,
            appliedLimits::add
        );

        currentLimitController.useShootingLimit();
        currentLimitController.useShootingLimit();
        currentLimitController.useNormalLimit();
        currentLimitController.useNormalLimit();

        assertEquals(List.of(40.0, 70.0), appliedLimits);
    }
}
