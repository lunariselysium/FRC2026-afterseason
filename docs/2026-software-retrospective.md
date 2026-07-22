# 2026 Robot Software Retrospective

## Season outcome

We finished third in qualification ranking, won one playoff match by a large margin, and then lost twice after a Raspberry Pi associated with the turret vision system failed to boot and the robot lost reliable field position.

That ending is painful, but it should not erase what the software demonstrated. This was not a robot program that only mapped buttons to motors. It grew into a field-aware, characterized, closed-loop system with autonomous paths, vision pose estimation, moving-shot compensation, mechanism recovery, and a meaningful deterministic test suite.

The most important lesson is not simply that a Raspberry Pi failed. Hardware will eventually fail. The deeper lesson is that one failed computer was allowed to remove too much confidence from the robot at once, and one autonomous recovery path could wait indefinitely for vision to return. The software did not cause the Pi to fail, but its failure policy amplified the effect of that failure.

## Executive assessment

The strongest parts of the codebase are its mechanism control, scoring mathematics, safety checks, and progressive extraction of pure logic into tests. The weakest part is system-level resilience: camera and coprocessor health were observable in pieces, but there was no unified pre-match readiness check, no explicit degraded operating mode, and no bounded fallback when autonomous relocalization could not obtain vision.

In short:

- The team learned control theory and applied it instead of relying only on trial-and-error percentages.
- The scoring system combined robot pose, robot velocity, alliance, target selection, vision trim, shot curves, and mechanism readiness.
- Mechanisms generally had current limits, travel limits, homing rules, and safe disabled behavior.
- Pure calculations and state machines were tested unusually well for a young FRC codebase: the repository contains 103 `@Test` methods in 25 test classes.
- The largest remaining gap is not another PID gain. It is designing and testing the robot as a distributed system whose sensors, networks, and coprocessors can disappear during a match.

## What went right

### 1. Characterization was treated as part of development

The repository contains SysId support for:

- drivetrain translation, steering, and rotation in `CommandSwerveDrivetrain`;
- turret heading, pitch, and flywheel in `Turret` and its component classes;
- intake deployment in `Intake`.

This is important because it changes tuning from "increase a number until it looks better" into a repeatable measurement process. The code records mechanism position, velocity, voltage, and current, and the mechanism routines include time limits and safety preconditions.

Particularly good details include:

- turret pitch SysId refuses to move before homing;
- turret heading SysId refuses to move when heading motion is not trusted;
- intake deploy SysId checks homing state and applies travel limits;
- SysId routines are intentionally not bound to competition controls;
- the drivetrain uses the vendor-supported translation, steer, and rotation characterization requests.

For newcomers, this is an excellent example of why SysId is not a magic tuning button. The useful workflow is: establish units and sensor signs, characterize, inspect the data, transfer gains, constrain the mechanism, and validate on the real robot.

### 2. Motion was shaped instead of commanded instantaneously

The software uses several forms of motion profiling:

- `MotionMagicVelocityVoltage` controls flywheel acceleration and jerk;
- `MotionMagicVoltage` controls intake deploy position;
- the intake switches to a slower Motion Magic profile during automatic scoring retraction;
- turret heading and pitch implement application-level velocity and acceleration profiles before applying feedback and feedforward.

This is good engineering for mechanisms with inertia, belts, racks, hard stops, and limited current. It reduces shock loads and makes mechanism timing more repeatable. It also makes readiness checks more meaningful because the mechanism approaches a planned state rather than being slammed toward a setpoint.

### 3. Feedforward was used in several appropriate ways

The code demonstrates that "feedforward" is not one single technique:

- flywheel control uses characterized `kS`, `kV`, and `kA` values in the CTRE closed-loop slot;
- the flywheel adds voltage for feeder load and an additional pass-shot policy;
- turret pitch uses gravity compensation, velocity feedforward, and proportional feedback;
- turret heading uses static and velocity feedforward plus proportional feedback;
- PathPlanner passes calculated wheel-force feedforwards to the drivetrain;
- intake position control includes a direction-aware stow-assist voltage.

The flywheel design is especially instructive. The controller handles the base velocity model, while application-level feedforward accounts for disturbances that are not captured by the unloaded model, such as feeding a game piece. That is a sound separation of responsibilities.

### 4. Scoring was modeled as a complete pipeline

`ScoringCalculator` and the scoring commands do considerably more than point the turret at a fixed coordinate. The pipeline includes:

- alliance-aware hub and pass targets;
- automatic selection between hub and pass behavior;
- the turret pivot's field position rather than only the robot center;
- robot-pose prediction before release;
- field-relative robot velocity and time-of-flight compensation;
- turret-forward vision correction when a fresh observation is available;
- interpolated shot maps and optional extrapolation;
- a live distance multiplier for field tuning;
- separate pitch, flywheel, and distance-validity results.

This is the right conceptual model for shooting while moving. The code recognizes that aiming is not just a heading problem: target location, robot motion, release delay, shot flight time, and mechanism setpoints are coupled.

### 5. Feeding was protected by readiness logic

`ScoreCommand` does not immediately run the feeder when the driver requests a shot. It checks:

- whether the requested distance is allowed;
- turret heading readiness;
- turret pitch readiness;
- flywheel minimum velocity;
- a debounce period before feeding.

`FeedControlStateMachine` can also reduce feed output when flywheel speed drops and require recovery before returning to full feed. This is a strong example of converting a mechanism interaction into an explicit state machine that can be unit tested.

The later cross-controller handoff changes also show good attention to operator experience: a warmup request can transfer to scoring without unnecessarily stopping the flywheel.

### 6. Mechanism code anticipated real failure modes

The code contains several practical reliability features:

- current limits on every major mechanism;
- homing current thresholds with minimum run times and debounce windows;
- homing timeouts for intake deploy and turret pitch;
- software travel limits during normal control and SysId;
- intake hard-stop capture and encoder re-zeroing;
- turret through-bore encoder monitoring;
- fallback to the Kraken internal encoder if the through-bore disconnects after a valid startup;
- feeder jam detection using requested output, current, and velocity;
- a manual unjam procedure coordinating feeder reversal and intake movement;
- drivetrain current limiting during shooting to protect available voltage.

These features came from treating the physical robot as imperfect. That habit is more valuable than any individual constant.

### 7. Autonomous assets were treated as code

The repository validates PathPlanner assets with tests and generates the right-side `R.2` autonomous routine by mirroring the left-side source. This reduces hand-edited divergence between symmetric routines and checks that paths can produce meaningful trajectories.

The autonomous chooser also has a safe "Do Nothing" fallback if AutoBuilder configuration fails.

### 8. Telemetry load was considered

Dashboard publishing is divided into phases with `TelemetryRateLimiter` instead of sending every value on every 20 ms loop. This is a subtle but important improvement: observability should not overload NetworkTables or steal time from control code.

The code also distinguishes operator-facing status from lower-level mechanism measurements, including readiness, jam state, homing state, encoder fallback, pose fusion mode, and shot solution values.

### 9. Pure logic was extracted and tested

The codebase contains 103 test methods across 25 test classes. Strong examples include:

- scoring interpolation, target selection, motion compensation, and field geometry;
- turret heading wrap and encoder math;
- flywheel readiness and feedforward math;
- vision geometry and relocalization sample consistency;
- feed state transitions;
- intake homing and manual recovery calculations;
- feeder jam qualification and unjam timing;
- telemetry rate limiting;
- PathPlanner asset validation and mirroring.

This is the right boundary for unit testing FRC code: move deterministic math and state transitions out of hardware classes, then test them aggressively.

## What could be improved

### Priority 0: make perception failure bounded and survivable

The most important concrete finding is in `VisionRelocalizeCommand`.

The command stops the drivetrain and finishes only when `vision.isPoseRelocalized()` becomes true. After `kRelocalizationWarningSeconds`, it reports a warning, but it has no timeout and no fallback result. In an autonomous sequence, a failed Pi, disconnected camera, missing tags, bad exposure, or inconsistent estimates can therefore stop the robot for the rest of autonomous.

This should become a bounded recovery policy. For example:

1. Wait for a stable vision pose for a short, explicit maximum duration.
2. If vision succeeds, reset pose and continue the normal routine.
3. If vision fails, keep the wheel-and-gyro odometry estimate, mark the pose as degraded, and run a conservative fallback action.
4. If the next planned motion is unsafe without localization, stop intentionally and report a latched fault rather than waiting forever for a resource that may never return.

The correct fallback depends on field geometry, but the wait itself should always be bounded.

The warning text also says it is waiting for a "stable multi-tag vision pose," while `kRelocalizationMinimumTagCount` is currently `1`. Either the requirement or the operator message should be corrected.

### Priority 0: add a real coprocessor and camera readiness system

The code publishes `PhotonCamera.isConnected()` and cumulative estimate counts, but that is not yet a match-readiness system.

Add a central health model with at least:

- expected Pi heartbeat and boot completion;
- camera connected state;
- age of the last received frame;
- age of the last accepted pose;
- current frame rate or result rate;
- time synchronization health;
- whether the camera's transform and pose fusion are enabled;
- a latched "not ready at enable" event;
- a single driver-visible summary such as `READY`, `DEGRADED`, or `FAULT`.

Run this check while disabled and make the result impossible to miss. A Driver Station message buried in logs is not enough. Use LEDs, an Elastic status panel, and controller feedback with distinct meanings.

The physical topology should also be reviewed. Critical fixed-pose cameras should be distributed so one Pi, one power connection, or one network cable cannot remove all useful localization views. The turret camera may remain specialized for aiming, but localization should not depend on a single specialized computer.

### Priority 1: define an explicit degraded-navigation mode

Normal pose fusion already ignores unavailable cameras and can continue with drivetrain odometry, which is good. The rest of the system should expose that state deliberately.

A degraded mode could:

- continue field-relative driving from gyro and module odometry;
- disable only features that require high-confidence global pose;
- retain turret-relative or direct-tag aiming when available;
- choose a simpler autonomous continuation;
- lower driving speed near obstacles;
- tell the drivers exactly what assistance has been lost.

The key design principle is graceful reduction of capability rather than a binary "fully autonomous or unusable" state.

### Priority 1: stop manually driving a command lifecycle

`RobotContainer.updatePathPlannerAutoScore()` calls `scoreCommand.initialize()`, `execute()`, and `end()` directly. The comment explains why: scheduling `ScoreCommand` would conflict with the enclosing PathPlanner command's requirements.

The motivation is understandable, but manually invoking command lifecycle methods bypasses the scheduler's requirement, interruption, and composition guarantees. It also makes testing the behavior harder.

Next season, model autonomous scoring as a command or superstructure state that PathPlanner can compose normally. Possible designs include:

- a dedicated long-running named command with explicit deadline/interruption behavior;
- a superstructure subsystem that owns the scoring state machine while commands only request modes;
- an autonomous routine composed in Java where scoring commands and path commands share requirements intentionally.

The scheduler should remain the sole owner of command lifecycle.

### Priority 1: create a central fault surface

`docs/robot-todo.md` already recognizes this need. The repository reports individual warnings and publishes many booleans, but drivers need one prioritized answer during a match.

Create a fault manager that records:

- severity;
- subsystem;
- active versus latched state;
- first occurrence time;
- operator action, if any;
- whether the fault blocks a mechanism or only degrades performance.

Examples that deserve prominent status are turret absolute encoder missing at startup, intake or pitch homing timeout, camera/Pi not ready, CAN device missing, persistent feeder jam, brownout, and autonomous relocalization fallback.

There is also documentation drift: intake and turret pitch homing timeouts now exist in code, while `robot-todo.md` still lists adding them. Close completed items or turn the file into a tracked checklist with status and evidence.

### Priority 2: reduce coordination logic in `RobotContainer`

`RobotContainer` has grown beyond bindings and composition. It now owns PathPlanner mechanism request state, autonomous scoring lifecycle, intake assistance, roller policy, jam rumble, manual unjam coordination, shot tuning controls, and target pointing.

This worked, but it makes control priority difficult to reason about. A `Superstructure` or coordinator layer would provide explicit modes such as:

- idle;
- intake;
- warmup;
- score;
- autonomous score;
- unjam;
- homing;
- characterization.

Commands would request a mode, and one state machine would resolve priority and apply subsystem goals. This would also make the cross-controller handoff and manual unjam interactions easier to test.

### Priority 2: calculate one shared scoring solution

Very similar scoring calculations appear in `ScoreCommand`, `ShotPrepCommand`, `ShotAimCommand`, `RobotContainer.pointTurretAtIntendedTarget()`, and `ScoringTelemetry`.

They currently agree, but duplicated pipelines tend to drift. Extract a shared immutable `ScoringSolution` containing:

- predicted robot pose;
- selected target and mode;
- compensated target;
- distance and visual trim;
- pitch and flywheel setpoints;
- feed permission;
- data freshness or confidence.

Commands and telemetry should consume the same solution rather than recomputing it independently.

### Priority 2: expand tests from pure logic to failure scenarios

The pure unit tests are a major strength. The next step is system behavior under failure.

Add tests or simulation scenarios for:

- one Pi missing before enable;
- a camera disconnecting mid-auto;
- no acceptable vision result during relocalization;
- stale PhotonVision frames;
- gyro drift or a pose jump;
- controller handoff while scoring and unjamming overlap;
- command interruption and subsystem requirement conflicts;
- homing sensors that never reach their thresholds;
- brownout or severe battery sag;
- autonomous asset generation from a clean checkout.

The repository already uses Hoot replay for timestamps and joysticks. That can become the start of log-replay regression tests based on real match failures.

### Priority 2: make calibration explicit and persistent

Several values are field-tuned or startup-sensitive:

- turret absolute encoder offset and required startup heading;
- camera transforms;
- shot maps and time-of-flight assumptions;
- the runtime shot-distance multiplier;
- hard-stop thresholds and homing currents.

For each calibration, record:

- physical measurement method;
- date and robot configuration;
- units and sign convention;
- expected valid range;
- how to verify it before a match.

The shot-distance multiplier resets at reboot and can increase without an upper bound. Give it a safe range, show it prominently, log every change, and provide an intentional way to save an accepted value back into configuration.

### Priority 3: make build-time generated assets less surprising

`compileJava` depends on a Gradle task that writes mirrored PathPlanner files into `src/main/deploy` and deletes obsolete generated mirror paths there. This is clever and the assets are tested, but compiling can mutate the source tree.

A cleaner long-term design is either:

- generate into the build directory and package the generated deploy assets from there; or
- check generated assets into source control and use a verification task that fails when they are stale.

Avoid doing both generation and source ownership at the same time.

### Priority 3: improve naming, size, and history quality

Several core classes are large: `Intake`, `TurretHeading`, `RobotContainer`, `Vision`, and `ScoringCalculator` are each hundreds of lines. They contain useful logic, but smaller responsibilities would make review and onboarding easier.

The commit history also contains messages such as `hm`, `yeet`, `fingers crossed`, and `may luck be on our side`. Those messages honestly reflect build-season emotion, but they make later diagnosis much harder. The strongest recent messages—such as `Fix cross-controller shot handoff priority`—are much more useful.

Use short imperative messages that describe behavior and subsystem, for example:

- `Bound vision relocalization wait in autonomous`
- `Add turret Pi readiness alert`
- `Tune intake homing current threshold`
- `Preserve flywheel speed during controller handoff`

Good history is part of the robot's documentation.

## A stronger development process for next season

### Before mechanisms are complete

- Define coordinate frames, units, motor signs, and sensor signs in a short design document.
- Create subsystem IO boundaries so pure logic and hardware access can be tested separately.
- Establish a central fault model and logging format before faults multiply.
- Decide which functions must survive loss of vision, one CAN device, or one coprocessor.

### During mechanism bring-up

- Verify current limits and neutral modes first.
- Verify sensor direction with motors mechanically safe.
- Home or establish an absolute reference.
- Run SysId at conservative voltage.
- Transfer gains and validate step responses;
- add travel limits and timeout behavior;
- record the final calibration and test evidence.

### During integration

- Use explicit superstructure modes and command requirements.
- Build failure injection into simulation and pit checks.
- Test controller priority as a state machine rather than only by feel.
- Rehearse rebooting each Pi, disconnecting each camera, and losing NetworkTables.

### Before every event

- Boot the complete robot from cold power several times.
- Confirm both Pis, all cameras, time synchronization, and result freshness.
- Confirm every required CAN device and absolute sensor.
- Confirm odometry-only driving and the degraded vision mode.
- Run a short autonomous smoke test from a clean deploy.
- Save logs and note the exact software commit.
- Keep a tested fallback autonomous routine that uses fewer external dependencies.

## Suggested newcomer video series

The best version of this series should show the learning path, including failed approaches and why the final design is better.

### Episode 1: The robot program as a system

Explain the roboRIO, CAN buses, motor controllers, sensors, Driver Station, NetworkTables, Raspberry Pis, PhotonVision, and field network. End with the idea that every connection is a potential failure boundary.

Repository examples: `Robot`, `RobotContainer`, subsystem construction, vendordeps.

### Episode 2: Units, gearing, signs, and coordinate frames

Show how a motor rotation becomes turret degrees or intake travel, why sign mistakes are dangerous, and how WPILib field coordinates differ from robot-relative coordinates.

Repository examples: `Constants`, `TurretHeadingMath`, `TurretPitchMath`, `VisionMath`.

### Episode 3: Safe mechanism bring-up

Cover current limits, neutral mode, open-loop tests, soft limits, homing, timeouts, and how to keep the robot mechanically safe.

Repository examples: `Intake`, `TurretPitch`, `TurretHeading`.

### Episode 4: SysId without mystery

Demonstrate quasistatic and dynamic tests, explain `kS`, `kV`, and `kA`, show what good and bad data look like, and explain why mechanisms must be characterized in meaningful units.

Repository examples: drivetrain, turret, flywheel, and intake SysId routines.

### Episode 5: Feedback, feedforward, and motion profiling

Compare pure proportional control, feedforward, and profiled setpoints. Show why gravity compensation is different from flywheel velocity feedforward and why Motion Magic reduces shock.

Repository examples: turret pitch, turret heading, `TurretFlywheel`, intake Motion Magic.

### Episode 6: Command-based ownership and state machines

Explain requirements, interruption, default commands, trigger semantics, and why manually calling command lifecycle methods is risky. Introduce a superstructure state machine.

Repository examples: `ScoreCommand`, `ShotPrepCommand`, `FeedControlStateMachine`, and the autonomous scoring coordination code.

### Episode 7: Building a field-aware scoring solution

Start with pointing at a fixed target, then add turret offset, alliance selection, pass targets, robot motion prediction, projectile lead, shot curves, and readiness interlocks.

Repository examples: `ScoringCalculator`, `ScoreCommand`, `ShotDistanceTuning`.

### Episode 8: Vision pose estimation

Explain AprilTags, camera transforms, ambiguity, timestamped measurements, standard deviations, multi-camera fusion, stale data, and why a turret-mounted camera needs a dynamic transform.

Repository examples: `Vision`, `VisionMath`, `VisionPoseRelocalizer`.

### Episode 9: Designing for camera and Pi failure

Recreate the playoff failure in simulation or replay. Show the difference between an unbounded wait and a bounded degraded-mode fallback. Cover power, networking, heartbeats, and pre-match health checks.

Repository examples: `VisionRelocalizeCommand`, camera telemetry, and the future fault manager.

### Episode 10: Autonomous paths as software assets

Cover PathPlanner configuration, named commands, command requirements, mirroring, asset validation, and safe fallback autos.

Repository examples: `build.gradle`, `PathPlannerAssetTest`, bump crossing commands.

### Episode 11: Testing robot code without the robot

Show how to extract math and state machines, write JUnit tests, use simulation, inject failures, and replay logs.

Repository examples: the 25 existing test classes and Hoot joystick/timestamp replay.

### Episode 12: Competition readiness and postmortems

Build a cold-boot checklist, failure matrix, logging checklist, commit discipline, and blameless postmortem process. Emphasize that reliability work is not separate from performance work.

Repository examples: this retrospective, `robot-todo.md`, and the Git history.

## Recommended video format

For each technical topic:

1. State the physical problem.
2. Show the simplest implementation.
3. Demonstrate its failure mode.
4. Introduce the relevant control or software concept.
5. Show the improved implementation.
6. Plot or log the result.
7. End with a pit checklist and one exercise for the viewer.

This format teaches judgment rather than only API calls.

## Final perspective

The season's software should be viewed as a successful first serious control-and-autonomy system with one major reliability lesson. The code shows real understanding of characterization, closed-loop control, feedforward, motion profiling, state machines, geometry, and deterministic testing. Those are difficult skills, and they were applied to a robot that ranked third and could win decisively.

The next step is to apply the same rigor to distributed-system failure: power, boot order, network health, stale data, bounded waits, degraded modes, and driver-visible faults. If that becomes a first-class design requirement next season, the team will not merely rebuild this year's performance. It will make that performance dependable.

## Archival validation note

This retrospective was produced through static code and Git-history review. Per repository instructions, no compile, simulation, test, or deploy command was run during the review. Before treating the branch as the final archival state, run from `rio/` with the 2026 WPILib toolchain:

```powershell
.\gradlew.bat test
.\gradlew.bat build
```

Record the results with the final commit hash and preserve representative match and failure logs alongside the season archive.
