package frc.robot.autonomous;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import org.junit.jupiter.api.Test;

class PathPlannerAssetTest {
    private static final double kFieldWidthMeters = 8.07;
    private static final double kMirrorTolerance = 1e-8;

    @Test
    void pathOneGeneratesAMeaningfulTrajectory() throws Exception {
        PathPlannerPath path = PathPlannerPath.fromPathFile("1");
        double durationSeconds = path
            .getIdealTrajectory(RobotConfig.fromGUISettings())
            .orElseThrow()
            .getTotalTimeSeconds();

        assertTrue(durationSeconds > 0.5);
    }

    @Test
    void lAutoResetsFromPathOneBeforeFollowingLaterPaths() throws Exception {
        PathPlannerPath firstPath = PathPlannerAuto.getPathGroupFromAutoFile("L.2").get(0);

        double startingX = firstPath.getStartingHolonomicPose().orElseThrow().getX();

        assertTrue(Math.abs(startingX - 4.464399399399399) < 1e-9);
    }

    @Test
    void rTwoAutoMirrorsEveryLTwoPathAndDirectionalCommand() throws Exception {
        RobotConfig robotConfig = RobotConfig.fromGUISettings();
        List<PathPlannerPath> leftPaths = PathPlannerAuto.getPathGroupFromAutoFile("L.2");
        List<PathPlannerPath> rightPaths = PathPlannerAuto.getPathGroupFromAutoFile("R.2");

        assertEquals(leftPaths.size(), rightPaths.size());
        for (int pathIndex = 0; pathIndex < leftPaths.size(); pathIndex++) {
            PathPlannerTrajectory leftTrajectory = leftPaths
                .get(pathIndex)
                .getIdealTrajectory(robotConfig)
                .orElseThrow();
            PathPlannerTrajectory rightTrajectory = rightPaths
                .get(pathIndex)
                .getIdealTrajectory(robotConfig)
                .orElseThrow();

            assertEquals(leftTrajectory.getStates().size(), rightTrajectory.getStates().size());
            for (int stateIndex = 0; stateIndex < leftTrajectory.getStates().size(); stateIndex++) {
                var leftState = leftTrajectory.getStates().get(stateIndex);
                var rightState = rightTrajectory.getStates().get(stateIndex);

                assertEquals(leftState.pose.getX(), rightState.pose.getX(), kMirrorTolerance);
                assertEquals(
                    kFieldWidthMeters,
                    leftState.pose.getY() + rightState.pose.getY(),
                    kMirrorTolerance
                );
                assertEquals(
                    0.0,
                    leftState.pose.getRotation().plus(rightState.pose.getRotation()).getRadians(),
                    kMirrorTolerance
                );
            }
        }

        String mirroredAuto = Files.readString(
            Path.of("src/main/deploy/pathplanner/autos/R.2.auto")
        );
        assertEquals(2, countOccurrences(mirroredAuto, "Bump Cross Forward Right"));
        assertFalse(mirroredAuto.contains("Bump Cross Forward Left"));
    }

    @Test
    void rTwoAutoStaysAtTheChooserRoot() throws Exception {
        String mirroredAuto = Files.readString(
            Path.of("src/main/deploy/pathplanner/autos/R.2.auto")
        );

        assertTrue(mirroredAuto.contains("\"folder\": null"));
    }

    private static int countOccurrences(String value, String target) {
        return (value.length() - value.replace(target, "").length()) / target.length();
    }
}
