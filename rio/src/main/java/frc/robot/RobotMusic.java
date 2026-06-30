// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MusicConstants;
import frc.robot.generated.TunerConstants;

public class RobotMusic {
    private final Orchestra orchestra = new Orchestra();
    // Keep the duplicate TalonFX handles alive for as long as the Orchestra owns them.
    private final List<TalonFX> instruments = new ArrayList<>();
    private final File musicFile = new File(Filesystem.getDeployDirectory(), MusicConstants.kMusicFileName);

    private boolean musicLoaded;
    private boolean playRequested;
    private boolean reportedPlayFailure;
    private boolean reportedStopFailure;

    public RobotMusic() {
        AudioConfigs audioConfigs = new AudioConfigs()
            .withAllowMusicDurDisable(true);

        for (MusicConstants.MusicTrackAssignment assignment : MusicConstants.kMusicTrackAssignments) {
            int canId = assignment.motorCanId();
            int trackNumber = assignment.trackNumber();
            TalonFX instrument = new TalonFX(canId, TunerConstants.kCANBus);
            instruments.add(instrument);
            reportIfBad(
                "Failed to allow disabled music on TalonFX " + canId,
                instrument.getConfigurator().apply(audioConfigs)
            );
            reportIfBad(
                "Failed to add TalonFX " + canId + " as a music instrument on track " + trackNumber,
                orchestra.addInstrument(instrument, trackNumber)
            );
        }

        musicLoaded = reportIfBad(
            "Failed to load music from " + musicFile.getPath(),
            orchestra.loadMusic(musicFile.getAbsolutePath())
        );
    }

    public void update() {
        SmartDashboard.putBoolean("RobotMusic/Loaded", musicLoaded);
        SmartDashboard.putBoolean("RobotMusic/Playing", orchestra.isPlaying());

        if (!musicLoaded) {
            return;
        }

        if (!DriverStation.isDisabled()) {
            stopForEnabledMode();
            return;
        }

        if (playRequested) {
            return;
        }

        if (orchestra.isPlaying()) {
            playRequested = true;
            return;
        }

        StatusCode status = orchestra.play();
        if (status.isOK()) {
            playRequested = true;
            reportedPlayFailure = false;
            return;
        }

        if (!reportedPlayFailure) {
            DriverStation.reportWarning("Failed to play robot music: " + status, false);
            reportedPlayFailure = true;
        }
    }

    private void stopForEnabledMode() {
        if (!playRequested) {
            return;
        }

        reportStopStatus(orchestra.stop());
        playRequested = false;
    }

    private boolean reportIfBad(String message, StatusCode status) {
        if (status.isOK()) {
            return true;
        }

        DriverStation.reportWarning(message + ": " + status, false);
        return false;
    }

    private void reportStopStatus(StatusCode status) {
        if (status.isOK()) {
            reportedStopFailure = false;
            return;
        }

        if (!reportedStopFailure) {
            DriverStation.reportWarning("Failed to stop robot music: " + status, false);
            reportedStopFailure = true;
        }
    }
}
