// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

public enum BumpCrossingDirection {
    FORWARD_LEFT(1.0, 1.0),
    FORWARD_RIGHT(1.0, -1.0),
    BACKWARD_LEFT(-1.0, 1.0),
    BACKWARD_RIGHT(-1.0, -1.0);

    private static final double kDiagonalComponent = 1.0 / Math.sqrt(2.0);

    private final double xSign;
    private final double ySign;

    BumpCrossingDirection(double xSign, double ySign) {
        this.xSign = xSign;
        this.ySign = ySign;
    }

    public double getVelocityXMetersPerSecond(double speedMetersPerSecond) {
        return Math.abs(speedMetersPerSecond) * xSign * kDiagonalComponent;
    }

    public double getVelocityYMetersPerSecond(double speedMetersPerSecond) {
        return Math.abs(speedMetersPerSecond) * ySign * kDiagonalComponent;
    }
}
