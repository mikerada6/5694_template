# FRC Swerve Drive Template - Setup Guide

**Purpose:** Instructions for cloning and customizing this template for a new FRC season.

**Target Audience:** Team mentors and veteran programmers starting a new season.

---

## 🎯 What is This Template?

This is a **year-to-year reusable codebase** for FRC swerve drive robots. It includes:

✅ Complete MAXSwerve drivetrain control
✅ PhotonVision AprilTag pose estimation
✅ AdvantageKit logging and replay
✅ PathPlanner autonomous integration
✅ Comprehensive safety features
✅ Student-friendly documentation

**What's NOT included:**
- ❌ Game-specific subsystems (intake, shooter, climber, etc.)
- ❌ Game-specific autonomous routines
- ❌ Current season field layout (you must configure)

---

## 📋 Required Updates When Starting New Season

### 1. Update Team Information

**File:** `src/main/java/frc/robot/Robot.java` (lines 38-41)

```java
// BEFORE (Template)
Logger.recordMetadata("ProjectName", "FRC-Swerve-Template");
Logger.recordMetadata("Team", "[Team XXXX] [Team Name]");
Logger.recordMetadata("Game", "[Current Season Game]");

// AFTER (Your Team)
Logger.recordMetadata("ProjectName", "FRC5684-2026");
Logger.recordMetadata("Team", "5684 Titans of Tech");
Logger.recordMetadata("Game", "2026 Game Name");
```

---

### 2. Configure AprilTag Field Layout

**File:** `src/main/java/frc/robot/RobotContainer.java` (line ~780)

```java
// BEFORE (Template - will throw error!)
throw new UnsupportedOperationException(
  "Field layout not configured! Update RobotContainer.loadFieldLayout() with current season field."
);

// AFTER (Current Season)
return AprilTagFieldLayout.loadField(AprilTagFields.k2026GameName);
```

**Available field layouts:**
- `k2024Crescendo`
- `k2023ChargedUp`
- `k2025Reefscape`
- `k2026...` (check WPILib docs for current year)

---

### 3. Update Field Constants

**File:** `src/main/java/frc/robot/constants/FieldConstants.java`

This file contains **EXAMPLE positions only**. You must:

1. Download field drawings from FRC game manual
2. Identify scoring locations and their (x, y) coordinates
3. Replace example constants with actual measurements
4. Rename constants to match game terminology

**Example transformation:**

```java
// TEMPLATE EXAMPLES (generic)
public static final Pose2d kBlueScoringLeft = new Pose2d(2.0, 5.5, ...);
public static final Pose2d kBlueScoringCenter = new Pose2d(2.0, 4.0, ...);

// YOUR GAME (e.g., 2026 with "Speaker" scoring location)
public static final Pose2d kBlueSpeakerLeft = new Pose2d(1.5, 5.8, ...);
public static final Pose2d kBlueSpeakerCenter = new Pose2d(1.5, 4.0, ...);
public static final Pose2d kBlueAmpZone = new Pose2d(0.5, 7.2, ...);
```

---

### 4. Update WPILib and Vendor Dependencies

**Update WPILib Suite:**
1. Download latest WPILib installer for current year
2. Install on all development machines
3. Restart VS Code

**Update Vendor Dependencies:**
1. Open VS Code command palette (Ctrl+Shift+P / Cmd+Shift+P)
2. Run: `WPILib: Manage Vendor Libraries`
3. Select: `Check for updates (online)`
4. Update all libraries to current year versions

**Files auto-updated:**
- `vendordeps/*.json` (REVLib, PathPlanner, PhotonLib, etc.)
- `build.gradle` (GradleRIO version)

**Manual file updates:**
- `settings.gradle` - Line 7: `String frcYear = '2026'`
- `.wpilib/wpilib_preferences.json` - Lines 4-5: Update year and team number

---

### 5. Calibrate Robot Hardware

**After building new robot, measure and update:**

**File:** `src/main/java/frc/robot/constants/DriveConstants.java`

```java
// Measure with ruler/calipers
kTrackWidth = 0.XXX;  // Distance between left and right wheels (meters)
kWheelBase = 0.XXX;   // Distance between front and rear wheels (meters)

// Calibrate with Phoenix Tuner
kFrontLeftChassisAngularOffset = X.XX;
kFrontRightChassisAngularOffset = X.XX;
kBackLeftChassisAngularOffset = X.XX;
kBackRightChassisAngularOffset = X.XX;

// Verify CAN IDs match physical wiring
kFrontLeftDrivingCanId = X;
kFrontLeftTurningCanId = X;
// ... etc for all 8 motor controllers
```

**File:** `src/main/java/frc/robot/constants/VisionConstants.java`

```java
// Measure camera position relative to robot center
kCameraToRobotX = 0.XXX;  // Forward (meters)
kCameraToRobotY = 0.XXX;  // Left (meters)
kCameraToRobotZ = 0.XXX;  // Up (meters)

// Measure camera angles
kCameraPitch = X.X;  // Degrees (tilt up/down)
kCameraYaw = X.X;    // Degrees (rotate left/right)
kCameraRoll = X.X;   // Degrees (rotate around lens axis)
```

---

### 6. Reset PID Tuning Values

**DO NOT carry over PID values from previous robots!**

Every robot has different mass, friction, and motor characteristics.

**File:** `src/main/java/frc/robot/constants/TeleopConstants.java`

```java
// Reset to safe starting values
kHeadingLockP = 5.0;
kHeadingLockI = 0.0;
kHeadingLockD = 0.0;

kAutoAimP = 5.0;
kAutoAimI = 0.0;
kAutoAimD = 0.0;

// ... same for all PID controllers
```

Then follow: `docs/PID_TUNING_GUIDE.md` to tune for your new robot.

---

### 7. Add Game-Specific Code

**Create new subsystems for game mechanisms:**

```
src/main/java/frc/robot/subsystems/
├── DriveSubsystem.java       ← KEEP (reusable)
├── VisionSubsystem.java      ← KEEP (reusable)
├── IntakeSubsystem.java      ← CREATE (game-specific)
└── ScorerSubsystem.java      ← CREATE (game-specific)
```

**Create command folder for game:**

```
src/main/java/frc/robot/commands/
├── DriveCommands.java        ← KEEP (reusable)
└── [gamename]/               ← CREATE folder
    ├── IntakeCommands.java   ← Game-specific commands
    └── ScoringCommands.java  ← Game-specific commands
```

**Register PathPlanner named commands:**

Edit `RobotContainer.registerNamedCommands()` (line ~885):

```java
NamedCommands.registerCommand("Intake Game Piece", intakeSubsystem.runIntake());
NamedCommands.registerCommand("Score Game Piece", scorerSubsystem.score());
```

---

### 8. Update Documentation

**Update all guide footers:**

Find and replace in all `docs/*.md` files:
- `[Current Season]` → `2026 Game Name`
- `[Team XXXX]` → `5684`
- `Last Updated: YYYY-MM-DD` → Today's date

**Update README.md:**
- Line ~173: Update game year
- Line ~174: Update team number and name

---

## 🧪 Testing Workflow

Follow these guides **in order** after customizing template:

1. **`docs/START_OF_SEASON_GUIDE.md`** - Master workflow (Week 1-4)
2. **`docs/PID_TUNING_GUIDE.md`** - Tune rotation controllers (Week 1-2)
3. **`docs/VISION_GUIDE.md`** - Test vision system (Week 2-3)
4. **`docs/AUTO_TESTING_GUIDE.md`** - Create autonomous (Week 3-4)

**Critical: Do PID tuning BEFORE vision testing!**

---

## ✅ Final Checklist

Before considering template customization complete:

- [ ] Team metadata updated in `Robot.java`
- [ ] AprilTag field layout configured in `RobotContainer.java`
- [ ] Field constants replaced with actual game measurements
- [ ] WPILib and vendor dependencies updated to current year
- [ ] Robot dimensions measured and entered in `DriveConstants.java`
- [ ] Camera position measured and entered in `VisionConstants.java`
- [ ] PID values reset to safe defaults (5.0, 0.0, 0.0)
- [ ] CAN IDs verified against physical wiring
- [ ] Game-specific subsystems created
- [ ] PathPlanner named commands registered
- [ ] Documentation footers updated with team/game/year
- [ ] Code compiles without errors: `./gradlew build`
- [ ] Code deploys to robot: `./gradlew deploy`

---

## 🚀 Ready to Go!

Once checklist complete, you have a **competition-ready template** customized for your team and season.

Follow the testing guides in `docs/` to get robot competition-ready!

---

**Questions?** See `docs/README.md` for complete documentation index.

**Last Updated:** 2026-01-18
**Template Version:** 1.0
**Maintained by:** FRC Template Contributors
