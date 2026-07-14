package frc.robot.subsystems;

import java.util.function.DoubleConsumer;

/** Applies drivetrain supply-current limits only when the requested mode changes. */
final class DriveCurrentLimitController {
    private final double normalSupplyCurrentLimitAmps;
    private final double shootingSupplyCurrentLimitAmps;
    private final DoubleConsumer applySupplyCurrentLimit;

    private double appliedSupplyCurrentLimitAmps = Double.NaN;

    DriveCurrentLimitController(
        double normalSupplyCurrentLimitAmps,
        double shootingSupplyCurrentLimitAmps,
        DoubleConsumer applySupplyCurrentLimit
    ) {
        this.normalSupplyCurrentLimitAmps = normalSupplyCurrentLimitAmps;
        this.shootingSupplyCurrentLimitAmps = shootingSupplyCurrentLimitAmps;
        this.applySupplyCurrentLimit = applySupplyCurrentLimit;
    }

    void useNormalLimit() {
        applyIfChanged(normalSupplyCurrentLimitAmps);
    }

    void useShootingLimit() {
        applyIfChanged(shootingSupplyCurrentLimitAmps);
    }

    private void applyIfChanged(double supplyCurrentLimitAmps) {
        if (Double.compare(appliedSupplyCurrentLimitAmps, supplyCurrentLimitAmps) == 0) {
            return;
        }

        applySupplyCurrentLimit.accept(supplyCurrentLimitAmps);
        appliedSupplyCurrentLimitAmps = supplyCurrentLimitAmps;
    }
}
