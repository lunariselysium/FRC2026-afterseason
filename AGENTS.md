# Repository Guidelines

## Project Structure & Module Organization

This repository contains the 2026 afterseason robot code. The roboRIO project lives in `rio/` and is a WPILib Java/GradleRIO project. Main robot code is under `rio/src/main/java/frc/robot/`, with subsystem classes in `rio/src/main/java/frc/robot/subsystems/`. Generated Phoenix Tuner swerve constants live in `rio/src/main/java/frc/robot/generated/`; treat these as generated configuration unless intentionally retuning. Deploy-time files belong in `rio/src/main/deploy/`. Vendor dependency JSON files are in `rio/vendordeps/`, and reference documentation is in `docs/`.

## Robot Hardware Notes

The robot has four major sections: an eight-Kraken swerve drivetrain on CAN IDs 1-8, an intake with deploy Falcon ID 11 and counterrotating roller Krakens IDs 12-13, a feeder with motors IDs 21-24, and a turret with lift ID 31, heading ID 32, pitch ID 33, and counterrotating flywheel Krakens. Four USB cameras are connected through two Raspberry Pi 5s running PhotonVision images: rear, left, right, and turret-forward.

## Build, Test, and Development Commands

Run commands from `rio/`:

- `./gradlew test` or `gradlew.bat test`: runs JUnit 5 tests.
- `./gradlew build`: builds the robot jar and runs checks.
- `./gradlew simulateJava`: starts WPILib desktop simulation when supported.
- `./gradlew deploy`: deploys to the roboRIO using the configured team number.

Agents should not compile, simulate, or deploy unless explicitly asked. Prefer making code changes, then ask the user to compile with the correct FRC toolchain and report errors.

## Coding Style & Naming Conventions

Use Java 17. Follow the existing WPILib command-based style: `RobotContainer` owns bindings, subsystem behavior lives in `subsystems/`, and constants live in nested `Constants` classes. Use 4-space indentation, `PascalCase` classes, `camelCase` methods/fields, and `k`-prefixed constants such as `kTurretMotorCanId`. Keep CAN IDs, gear ratios, inversion signs, and tuning gains centralized in constants.

## Testing Guidelines

JUnit 5 is configured in `build.gradle`, but no test tree currently exists. Add tests under `rio/src/test/java/` when logic is deterministic and hardware-independent. Name tests after the class or behavior under test, for example `TurretHeadingTest`.

## Commit & Pull Request Guidelines

Recent history uses short, imperative summaries such as `Temporarily tune down steer gains to prevent oscillations`. Keep commits focused and mention subsystem context when useful. Pull requests should describe behavior changes, list affected hardware IDs or subsystems, note required calibration, and include user-provided build/test results.
