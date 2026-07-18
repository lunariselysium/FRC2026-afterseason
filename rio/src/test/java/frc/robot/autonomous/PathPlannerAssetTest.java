package frc.robot.autonomous;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;

import org.junit.jupiter.api.Test;

class PathPlannerAssetTest {
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
}
