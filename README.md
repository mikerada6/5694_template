# FRC Swerve Drive Template

**A year-to-year reusable codebase for FIRST Robotics Competition swerve drive robots.**

[![WPILib](https://img.shields.io/badge/WPILib-2025+-blue)](https://docs.wpilib.org/)
[![Java](https://img.shields.io/badge/Java-17-orange)](https://www.oracle.com/java/)
[![License](https://img.shields.io/badge/License-BSD--3-green)](WPILib-License.md)

---

## 🎯 What is This?

This is a **production-ready robot codebase template** designed to be reused year-after-year by FRC teams. Clone it at the start of each season, customize for your game, and go!

### ✅ What's Included

- **Complete MAXSwerve Drive System** - REV Robotics swerve modules with field-relative control
- **PhotonVision Integration** - AprilTag-based pose estimation with safety filtering
- **AdvantageKit Logging** - Post-match replay and live telemetry via AdvantageScope
- **PathPlanner Autonomous** - Visual path planning with named command support
- **Comprehensive Safety Features** - Brownout detection, vision timeouts, emergency overrides
- **Student-Friendly Documentation** - Step-by-step guides for tuning, testing, and deployment

### ❌ What's NOT Included (You Add Each Year)

- Game-specific subsystems (intake, shooter, climber, etc.)
- Game-specific autonomous routines
- Field scoring positions (you must measure from game manual)
- Current season AprilTag layout (you must configure)

---

## 🚀 Quick Start

### 1. Clone This Template

```bash
git clone https://github.com/your-team/5694_template.git
cd 5694_template
```

### 2. Customize for Your Season

**Read:** [`TEMPLATE_SETUP.md`](TEMPLATE_SETUP.md) for complete customization instructions.

**Quick Checklist:**
- [ ] Update team info in `Robot.java`
- [ ] Configure AprilTag field layout in `RobotContainer.java`
- [ ] Replace field constants with actual game measurements
- [ ] Update WPILib and vendor dependencies to current year
- [ ] Measure and enter robot dimensions
- [ ] Calibrate swerve module offsets

### 3. Follow Testing Guides

1. **`docs/PID_TUNING_GUIDE.md`** - Tune rotation controllers (Week 1-2)
2. **`docs/VISION_GUIDE.md`** - Test vision system (Week 2-3)
3. **`docs/AUTO_TESTING_GUIDE.md`** - Create autonomous (Week 3-4)

**See:** [`docs/START_OF_SEASON_GUIDE.md`](docs/START_OF_SEASON_GUIDE.md) for complete workflow.

---

## 📚 Documentation

| Guide | Purpose | When to Use |
|-------|---------|-------------|
| [**TEMPLATE_SETUP.md**](TEMPLATE_SETUP.md) | Customize template for new season | Start of season (Week 0) |
| [**START_OF_SEASON_GUIDE.md**](docs/START_OF_SEASON_GUIDE.md) | Master 4-week roadmap | Start of season (Week 1-4) |
| [**ONBOARDING_GUIDE.md**](docs/ONBOARDING_GUIDE.md) | Student learning path | New students join team |
| [**PID_TUNING_GUIDE.md**](docs/PID_TUNING_GUIDE.md) | Tune rotation controllers | Week 1-2 (before vision!) |
| [**VISION_GUIDE.md**](docs/VISION_GUIDE.md) | Test AprilTag vision | Week 2-3 (after PID!) |
| [**AUTO_TESTING_GUIDE.md**](docs/AUTO_TESTING_GUIDE.md) | PathPlanner autonomous | Week 3-4 |
| [**SHUFFLEBOARD_GUIDE.md**](docs/SHUFFLEBOARD_GUIDE.md) | Dashboard configuration | Anytime |

**Full Documentation Index:** [`docs/README.md`](docs/README.md)

---

## 🏗️ Architecture

### Year-to-Year Reusable Core

**DO NOT MODIFY** these between seasons (unless fixing bugs):

- `DriveSubsystem.java` - Swerve drive physics and control
- `VisionSubsystem.java` - AprilTag pose estimation
- `DriveCommands.java` - Drive command factories
- `MAXSwerveModule.java` - Individual module control
- All constants files (update values, not structure)

### Game-Specific Additions

**ADD THESE** each season:

```
src/main/java/frc/robot/
├── subsystems/
│   ├── [Game]IntakeSubsystem.java      ← New
│   ├── [Game]ScorerSubsystem.java      ← New
│   └── [Game]ClimberSubsystem.java     ← New
│
└── commands/
    └── [gamename]/                      ← New folder
        ├── IntakeCommands.java          ← New
        └── ScoringCommands.java         ← New
```

Update `RobotContainer.java`:
- Button bindings for game mechanisms
- PathPlanner named commands
- Autonomous routines

---

## 🔧 Build & Deploy

```bash
# Build project (compile only)
./gradlew build

# Deploy to robot
./gradlew deploy

# Simulate robot code
./gradlew simulateJava

# Clean build artifacts
./gradlew clean
```

**Requirements:**
- Java 17 or higher
- WPILib (current year version)
- Vendor dependencies (auto-installed via `vendordeps/`)

---

## 🤖 Features

### Drive System
- MAXSwerve modules with NEO motors
- Field-relative and robot-relative modes
- Heading lock (auto-rotate while driving)
- Snap-to-angle (cardinal and 45° angles)
- Speed multiplier system (25%, 50%, 75%, 100%)

### Vision System
- Dual pose tracking (vision-fused + pure odometry)
- Multi-camera support
- Safety filtering (motion blur, distance, ambiguity)
- Classroom Mode toggle (5m threshold for testing)
- Vision timeout watchdog

### Autonomous
- PathPlanner integration
- Visual path editor
- Named command system
- Alliance-aware path flipping
- Test autos included

### Logging
- AdvantageKit structured logging
- Pose2d and SwerveModuleState[] arrays
- USB logging (/U/logs/) for post-match replay
- Live NetworkTables streaming
- AdvantageScope integration

### Safety
- Brownout detection and warnings
- Emergency override button
- Vision safety filters
- Module state validation
- Graceful vision fallback

---

## 🎮 Controller Layout

### Driver (Thrustmaster Joysticks on Ports 0 & 1)
- **Left Stick:** Translation (X/Y movement)
- **Right Stick:** Rotation (spin)
- **Left Stick Button 1:** Toggle field-relative mode
- **Right Stick Button 2:** Zero heading
- **Right Stick Button 3:** Emergency override

### Co-Driver (PlayStation 5 Controller on Port 2)
- **Triangle:** Heading lock to target
- **Circle:** Auto-aim at target
- **Cross:** Position at distance from target
- **Square:** X-stance (lock wheels)
- **L1/R1:** Speed control (25%, 50%, 75%, 100%)
- **L2/R2:** Snap to cardinal/45° angles
- **Touchpad:** Force vision reset
- **Options:** Reload test target

---

## 📦 Dependencies

This template uses:
- **WPILib** - FRC control system libraries
- **REVLib** - REV Robotics motor controllers
- **PathPlannerLib** - Autonomous path planning
- **PhotonLib** - AprilTag vision processing
- **AdvantageKit** - Structured logging and replay
- **Studica/NavX** - Gyroscope support

Vendor dependencies are in `vendordeps/` and auto-update via WPILib.

---

## 🤝 Contributing

This is a **template repository** meant to be cloned, not forked.

**To improve the template:**
1. Fork this repo
2. Make improvements to reusable code (Drive/Vision subsystems)
3. Submit PR with clear description
4. Do NOT include game-specific code in PRs

**Template Improvements Welcome:**
- Bug fixes in reusable code
- Documentation improvements
- New safety features
- Performance optimizations
- Better code comments

---

## 📄 License

This project is licensed under the WPILib BSD license. See [WPILib-License.md](WPILib-License.md).

---

## 🙏 Acknowledgments

- **WPILib** - FRC control system framework
- **REV Robotics** - MAXSwerve hardware and software
- **PhotonVision** - Vision processing library
- **Mechanical Advantage (6328)** - AdvantageKit logging framework
- **FRC Community** - Shared knowledge and support

---

## 📞 Support

**Documentation:** [`docs/README.md`](docs/README.md)
**Setup Guide:** [`TEMPLATE_SETUP.md`](TEMPLATE_SETUP.md)
**Issues:** Report bugs via GitHub Issues

---

**Template Version:** 1.0
**Last Updated:** 2026-01-18
**Maintained by:** FRC Template Contributors
**Tested with:** WPILib 2025+
