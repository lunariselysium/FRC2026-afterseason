package frc.robot.subsystems;

/** Describes the travel target and calibrated endpoint used while homing the intake deploy mechanism. */
record IntakeHomingPlan(
    double motionTargetPositionMotorRotations,
    double calibratedPositionMotorRotations,
    boolean targetDeployed,
    boolean usesNormalDeployMotionProfile
) {
    static IntakeHomingPlan toDeployedPosition(
        double deployedPositionMotorRotations,
        double deployedHardstopCaptureWindowMotorRotations
    ) {
        return new IntakeHomingPlan(
            deployedPositionMotorRotations + deployedHardstopCaptureWindowMotorRotations,
            deployedPositionMotorRotations,
            true,
            true
        );
    }

    static IntakeHomingPlan toStowedPosition(double deployedPositionMotorRotations) {
        // The unhomed integrated encoder can start at zero while the mechanism is deployed.
        // Command one full travel length past stow so Motion Magic always drives toward the hard stop.
        return new IntakeHomingPlan(
            -deployedPositionMotorRotations,
            0.0,
            false,
            true
        );
    }

    double targetPositionMotorRotations() {
        return calibratedPositionMotorRotations;
    }
}
