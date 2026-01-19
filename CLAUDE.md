# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an **FRC (FIRST Robotics Competition) robot codebase** for the 2025 Reefscape game, built on WPILib with Java. The architecture uses a **command-based framework** with swerve drive, vision-based pose estimation, and PathPlanner autonomous.

**Key Design Philosophy:**
- **Year-to-year reusable core**: `DriveSubsystem`, `VisionSubsystem`, and `DriveCommands` should NOT change between seasons
- **Game-specific changes**: Add new subsystems (intake, shooter, climber) and button mappings in `RobotContainer` each year
- **Comprehensive testing guides**: Follow `docs/` in order (PID → Vision → Auto) before competition

## Build System & Commands

This project uses **Gradle** with the WPILib GradleRIO plugin:

```bash
# Build the project
./gradlew build

# Deploy to robot (RoboRIO)
./gradlew deploy

# Run tests
./gradlew test

# Simulate robot code on desktop
./gradlew simulateJava

# Clean build artifacts
./gradlew clean
```

**Important Notes:**
- Team number is stored in `.wpilib/wpilib_preferences.json`
- The main robot class is `frc.robot.Main` (configured in `build.gradle`)
- Java 17 is required (sourceCompatibility set in `build.gradle`)

## Critical Architecture Patterns

### 1. Dual Pose Tracking System

The robot maintains **two separate pose trackers**:

- **Primary (Fused)**: `SwerveDrivePoseEstimator` - combines encoders + gyro + vision
  - Used for all driving commands and autonomous
  - Vision corrections snap robot back when AprilTags detected

- **Backup (Ghost)**: `SwerveDriveOdometry` - encoders + gyro only (no vision)
  - Compare drift between them to diagnose vision problems
  - Dashboard key: `Drive/PoseDrift` should stay < 0.5m

**Why this matters:** If pose drift is large, vision might be misconfigured (bad camera transform, wrong tags detected, reflections).

### 2. Vision Safety Pipeline

Vision measurements are **heavily filtered** before being trusted (`DriveSubsystem.java:517-613`):

1. **Motion blur check**: Reject if spinning > 720°/sec
2. **Multi-tag priority**: Always accept 2+ tag detections (very reliable)
3. **Single-tag filtering**:
   - Accept if correction < 1m (normal drift)
   - Reject if correction > 1m (likely noise/reflection)
4. **Dynamic trust levels**: Large corrections get higher trust when multi-tag

### 3. Dependency Injection Pattern

Vision uses the **VisionProvider interface** to allow graceful fallback:

```java
VisionProvider m_vision = createVisionProvider();  // In RobotContainer
```

- Returns `VisionSubsystem` if cameras available
- Returns `NoVisionProvider` if cameras missing (robot still drives, just without vision corrections)

### 4. Command Factories

All drive commands are **static factory methods** in `DriveCommands.java`:

```java
DriveCommands.joystickDrive(drive, xSupplier, ySupplier, rotSupplier)
DriveCommands.autoAimAtTarget(drive, targetPose)
DriveCommands.positionAtDistance(drive, target, distance)
```

**Benefits:**
- Keep command logic separate from subsystem physics
- Reusable across button bindings and autonomous
- Easy to test individual commands

### 5. Constants Organization

Constants are split by **domain** (not by type):

- `HardwareConstants.java`: CAN IDs, controller ports, physical measurements
- `DriveConstants.java`: Speed limits, PID values, safety thresholds
- `VisionConstants.java`: Camera transforms, AprilTag filtering
- `FieldConstants.java`: Game-specific field positions

**Update these each year:**
- Hardware IDs if wiring changes
- Camera transforms if camera relocated
- Field positions for new game

## Vendor Dependencies

Located in `vendordeps/`:

- **REVLib**: REV Robotics MAXSwerve modules
- **PathplannerLib**: Autonomous path planning
- **photonlib**: AprilTag vision (PhotonVision)
- **Studica**: NavX gyroscope (AHRS)
- **AdvantageKit**: Telemetry logging (optional)

## Year-to-Year Updates

When starting a new season, update these files **in this order**:

1. **Field layout** (`RobotContainer.java:621`):
   ```java
   AprilTagFieldLayout.loadField(AprilTagFields.k2026GameName)
   ```

2. **Camera transform** (`VisionConstants.java`):
   - Measure camera position relative to robot center
   - Update `kCameraToRobotX/Y/Z` and rotation angles

3. **Hardware constants** (`HardwareConstants.java`):
   - Verify CAN IDs match physical wiring
   - Calibrate chassis angular offsets with Phoenix Tuner

4. **Game subsystems** (`RobotContainer.java`):
   - Add intake, shooter, climber subsystems
   - Register PathPlanner named commands
   - Map buttons for game-specific actions

5. **Autonomous routines** (`RobotContainer.createAuto()`):
   - Create paths in PathPlanner GUI
   - Add to auto selector

**Files that should NOT change:**
- `DriveSubsystem.java` (core swerve control)
- `VisionSubsystem.java` (AprilTag processing)
- `MAXSwerveModule.java` (module control)

## Testing Workflow

**CRITICAL ORDER** (documented in `docs/README.md`):

1. **Week 1-2: PID Tuning** (`PID_TUNING_GUIDE.md`)
   - Tune rotation PID controllers
   - No vision hardware needed
   - Must complete before vision testing

2. **Week 2-3: Vision Testing** (`VISION_TESTING_GUIDE.md`)
   - Test vision commands safely
   - Requires PID tuning complete
   - Buttons: Triangle (heading lock), Circle (auto-aim), Cross (position)

3. **Week 3-4: Autonomous** (`AUTO_TESTING_GUIDE.md`)
   - Test PathPlanner paths
   - Requires PID + vision complete
   - Includes test autos: "Test Drive Forward", "Test L-Shape"

**Why this order?** Vision commands use PID controllers. If PID isn't tuned, vision tests fail even if vision works perfectly.

## PathPlanner Integration

Autonomous uses PathPlanner for path following:

- **Paths stored in**: `src/main/deploy/pathplanner/autos/`
- **Configuration**: `DriveSubsystem.configurePathPlanner()` (line 1096)
- **Named commands**: Register in `RobotContainer.registerNamedCommands()` (line 675)

**To add new auto:**
1. Create path in PathPlanner GUI (https://pathplanner.dev)
2. Save to `deploy/pathplanner/autos/`
3. Add to `createAuto()`:
   ```java
   auto.addOption("4 Piece Auto", new PathPlannerAuto("4 Piece Auto"));
   ```

## Dashboard Organization

Shuffleboard tabs (configured in `RobotContainer.configureShuffleboard()`):

- **Competition**: Minimal display for drivers (battery, speed, auto selector)
- **Drive**: Robot pose, speeds, field widget
- **Vision**: Camera status, AprilTag detection
- **Tuning**: Live PID tuning, module velocities
- **Commands**: Test buttons (force reset, speed presets, X-stance)

**Key telemetry:**
- `Drive/PoseDrift`: Distance between fused and pure odometry (< 0.5m = good)
- `Vision/Status`: Current vision state (accepted/rejected/timeout)
- `Drive/BatteryVoltage`: Battery health (> 10V during match)

## Common Pitfalls

1. **Gyro orientation**: If robot drives backward when forward commanded, gyro may be inverted
   - Fix: Check negation in `DriveSubsystem.getRotation2d()` (line 800)

2. **Vision jumping**: Robot "teleports" to wrong position
   - Likely: Wrong camera transform or seeing reflections
   - Debug: Check `Drive/PoseDrift` and `Vision/Status`

3. **Module offsets**: Robot drifts when driving straight
   - Fix: Recalibrate `kFrontLeftChassisAngularOffset` etc. with Phoenix Tuner

4. **Alliance flip**: Paths run backward on red alliance
   - Check: `configurePathPlanner()` has alliance flipper enabled (line 1131)

5. **Brownout during match**: Battery voltage < 6.5V
   - Causes: Too many motors, weak battery, poor connections
   - Monitor: `Drive/BatteryVoltage` on dashboard

## Pre-Match Checklist

Before each match (from `DriveSubsystem.java:88-93`):

1. Face robot **away from driver station**
2. Run `zeroHeading()` command (right joystick button 2)
3. Verify Field widget shows correct starting position
4. Check all 4 module velocities respond to joystick
5. Verify `Vision/[Camera]/Connected` is true
6. Check `Drive/PoseDrift` stays small (< 0.5m)

## Emergency Controls

- **Right joystick button 3**: Emergency override (cancels any running command, returns control to driver)
- **X-stance**: Square button on co-driver (locks wheels in X pattern - very hard to push)
- **Coast mode**: For pit crew to push robot easily when disabled

## Documentation

Comprehensive guides in `docs/`:
- `ONBOARDING_GUIDE.md`: For new students
- `START_OF_SEASON_GUIDE.md`: Master roadmap for competition prep
- `STUDENT_SETUP_CHECKLIST.md`: Pre-testing verification
- `SHUFFLEBOARD_GUIDE.md`: Dashboard configuration
