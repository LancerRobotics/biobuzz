# AGENTS.md

Guidance for AI coding agents (and humans) working in **biobuzz**, the LancerRobotics
FTC (FIRST Tech Challenge) robot code. Remote: `github.com/LancerRobotics/biobuzz`.

## What this repo is

An Android Studio project built on the official FTC Robot Controller SDK (v12.0.0).
It compiles into an Android app that runs on the robot's Control Hub.

| Module | Purpose | Edit it? |
|---|---|---|
| `TeamCode/` | All team code: OpModes, subsystems, autonomous paths, tuning | **Yes, all work goes here** |
| `FtcRobotController/` | FTC SDK app shell and sample OpModes | No. It's upstream SDK code |
| `build.common.gradle` | Shared Android build config from the SDK | Only if you really have to |
| `build.dependencies.gradle` | FTC SDK and Pedro Pathing dependency versions | Only for deliberate upgrades |

The `.github/CONTRIBUTING.md`, `README.md` and `TeamCode/.../readme.md` files are generic
FTC SDK boilerplate. They don't describe this team's code.

## Layout of TeamCode

Java root: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`

- `config/lancersBotConfig.java`: hardware-map device names (motors, servos, IMU, Pinpoint,
  Limelight). These strings must match the robot's configuration on the Driver Station.
  Always use these constants; never hard-code a device name.
- `subsystems/`: hardware wrappers (`drive`, `intake`, `outtake`, `turret`, `vision`) and
  `robot`, which builds them plus the goBILDA Pinpoint odometry.
  - `UKF.java`: a standalone unscented Kalman filter (X, Y, heading) that also outputs
    four-motor powers. It uses only `java.*`, with **no FTC SDK imports, on purpose**, so it
    runs and is testable on a plain JVM. Keep it that way.
  - `UKFSimulationHarness.java`: synthetic ground truth plus noisy sensors driving `UKF`.
    It has `main()` and `run(...)`.
  - `UKFProductionHarness.java`: wires `UKF` to the robot. The sensor-read and motor-write
    methods are `TODO` stubs.
- `lancersTeleopController.java`: shared TeleOp logic used by both alliance TeleOps.
- `opmodes/teleop/`: `lancersTeleOpBlue`, `lancersTeleOpRed` and `outtakeTuner`.
- `opmodes/auton/autonBlue/`, `opmodes/auton/autonRed/`: autonomous routines, one class
  per starting position/strategy (e.g. `autonBlueP` = parking).
- `pedroPathing/`: `Constants.java` (follower and Pinpoint localizer config) and the
  tuning OpModes. Retuning values here changes how every auton drives.

Tests: `TeamCode/src/test/java/...` holds local JVM JUnit 4 tests (currently `UKFSimulationTest`).

## Build and test

You need a JDK. Android Studio's bundled one works (set `JAVA_HOME` to
`<Android Studio install dir>/jbr`).

```bash
./gradlew :TeamCode:assembleDebug                                     # build the robot app
./gradlew :TeamCode:testDebugUnitTest                                 # all local JVM unit tests
./gradlew :TeamCode:testDebugUnitTest --tests "*UKFSimulationTest"    # one test class
```

On Windows, use `gradlew.bat` (or `./gradlew` from Git Bash). Test reports go to
`TeamCode/build/reports/tests/testDebugUnitTest/`. `build/` is git-ignored.

- **Anything that touches hardware can't be tested off the robot.** That means OpModes,
  subsystems, and anything using `HardwareMap`, `DcMotor` or `Telemetry`. Agents can only
  confirm that this code compiles. Say so explicitly instead of claiming it "works".
- Pure-math code (filters, kinematics, path geometry) should avoid SDK types so it can get
  JVM unit tests, like `UKF`.
- Deploying to the robot (Run ▶ in Android Studio with the Control Hub connected) is
  done by a human.

## Language level and constraints

- **Java 8** (`sourceCompatibility`/`targetCompatibility` 1.8), with Android
  `minSdkVersion 24`. Don't use Java 9+ APIs or syntax: no `var`, `List.of`,
  `OutputStream.nullOutputStream()`, records, text blocks or switch expressions. A
  desktop JDK will compile these fine, but they break or are missing on the robot.
  When compiling outside Gradle, use `javac --release 8` to catch them.
- The main control loop runs about every 20–50 ms. Avoid allocations in hot loops,
  blocking calls, `Thread.sleep` in iterative `OpMode`s, and heavy logging to
  `System.out`. Use `telemetry` for driver-visible output.
- In a `LinearOpMode`, loop on `opModeIsActive()`. Always stop motors in `stop()` or at
  the end of the routine.

## Conventions in this codebase

- **Class names are lowercase** for team classes (`drive`, `robot`, `autonBlueP`,
  `lancersTeleOpBlue`). Match the surrounding style when adding peers. Don't mass-rename
  existing classes: OpMode names show on the Driver Station, and renames break Git history.
- Register OpModes with `@TeleOp(name = ..., group = "TeleOp")` or
  `@Autonomous(name = ..., group = "Autonomous")`. The `name` is what drivers pick on
  the Driver Station, so keep names unique and readable.
- **Units differ between systems. Convert explicitly at the boundaries:**
  - Field and odometry poses (Pinpoint via `robot`, and Pedro Pathing) are in **inches**.
    `robot.getHeading()` returns **degrees**.
  - `follower.pose().heading()` (Pedro) returns **radians**. Auton pose literals are given
    in degrees through `PoseFactory.degrees()`.
  - `UKF` uses **meters and radians** internally.
- **Alliance mirroring:** Blue autons build poses with
  `PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75)`. Red/blue variants of a routine
  are separate classes, so a fix to one side usually belongs in its mirror too
  (e.g. `autonBlueSP` ↔ `autonRedSP`).
- **Auton → TeleOp pose handoff** uses static fields `robot.odoX`, `robot.odoY` and
  `robot.odoH` (default heading 90°). TeleOps read them in `init()`. Don't remove them
  without replacing that handoff.
- Autons use the Pedro Pathing Ivy scheduler: call `Scheduler.reset()` at the start, then
  `schedule(...)` commands, and call `follower.update()` plus `Scheduler.execute()` every
  loop iteration.

## Known gotchas

- The drivetrain is **mecanum** (`drive.driveMecanum(forward, strafe, rotate)`), but
  `UKF`'s built-in controller assumes **differential (tank) drive** with no strafing. That
  was the original spec. Before using `UKF` motor outputs on the real robot, decide
  whether to keep it tank-style or extend its state and control to mecanum.
- `drive.java` refers to the rear motors as `backLeftMotor`/`backRightMotor`, while
  `lancersBotConfig` calls them `REAR_*`. They are the same motors.
- The turret subsystem is currently commented out of use (see Git history).

## Git workflow

- `main` is the default branch. Do feature work on a branch (e.g. `feature/ukf`) and
  merge it through a pull request on GitHub.
- Teammates often have uncommitted work in progress. Before committing, check
  `git status` and stage only the files you changed. Never `git add -A` blindly, and
  never discard someone else's local changes.
- Keep commits focused, with short, imperative messages.
