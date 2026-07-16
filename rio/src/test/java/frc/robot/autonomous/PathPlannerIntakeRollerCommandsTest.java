package frc.robot.autonomous;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicBoolean;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj2.command.Command;

class PathPlannerIntakeRollerCommandsTest {
    @Test
    void startCommandRunsWithoutClaimingASubsystemRequirement() {
        AtomicBoolean rollersStarted = new AtomicBoolean();
        Command command = PathPlannerIntakeRollerCommands.startCommand(
            () -> rollersStarted.set(true)
        );

        command.initialize();

        assertTrue(rollersStarted.get());
        assertTrue(command.getRequirements().isEmpty());
    }
}
