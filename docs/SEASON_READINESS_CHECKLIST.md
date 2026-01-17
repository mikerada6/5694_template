# Season Readiness Checklist

**Purpose:** Verify codebase is ready for students at the start of a season.

**Use this before:** Handing codebase to students or starting a new season.

---

## ✅ Codebase Health

### Code Quality
- [x] No TODOs or FIXMEs in production code
- [x] All imports resolve (when built with FRC tools)
- [x] Constants have reasonable default values
- [x] All methods have clear javadoc comments
- [x] No "magic numbers" - all values in Constants.java

### Architecture
- [x] Clear separation: Reusable core vs game-specific
- [x] DriveSubsystem = Physics/state (year-to-year reusable)
- [x] DriveCommands = Behavior factories (year-to-year reusable)
- [x] VisionSubsystem = Pose estimation (year-to-year reusable)
- [x] RobotContainer = Button bindings (update each year)

### Safety Features
- [x] Emergency override button implemented (right stick button 3)
- [x] Brownout detection and warnings
- [x] Module state validation (prevents null/invalid states)
- [x] Vision timeout watchdog
- [x] Speed multiplier system (start at 25% for testing)

---

## 📚 Documentation Complete

### Core Guides
- [x] **docs/README.md** - Documentation index
- [x] **docs/ONBOARDING_GUIDE.md** - Student onboarding (freshmen + veterans)
- [x] **docs/START_OF_SEASON_GUIDE.md** - Master workflow
- [x] **docs/PID_TUNING_GUIDE.md** - PID tuning procedures
- [x] **docs/VISION_TESTING_GUIDE.md** - Vision testing procedures

### Cross-References
- [x] All guides link to each other correctly
- [x] All guides have "Back to Documentation Index" footer
- [x] Main README.md points to docs/ folder
- [x] No broken links (all files exist)

### Clarity for Both Audiences
- [x] Freshman path clearly defined in ONBOARDING_GUIDE
- [x] Veteran path clearly defined in ONBOARDING_GUIDE
- [x] Order is explicit: PID FIRST, then Vision
- [x] Safety warnings throughout guides
- [x] Common mistakes documented

---

## 🎯 Year-to-Year Update Checklist

**Use this when starting a new season:**

### Update Game Year References
- [ ] README.md game year
- [ ] All guide footers: "Game: 2026 GameName"
- [ ] All guide footers: "Last Updated: YYYY-MM-DD"
- [ ] Constants.FieldConstants (replace completely!)
- [ ] AprilTag field layout (e.g., k2025Reefscape → k2026GameName)

### Update Robot-Specific Values
- [ ] DriveConstants.kTrackWidth (measure new robot)
- [ ] DriveConstants.kWheelBase (measure new robot)
- [ ] DriveConstants.kFrontLeftChassisAngularOffset (calibrate)
- [ ] DriveConstants.kFrontRightChassisAngularOffset (calibrate)
- [ ] DriveConstants.kBackLeftChassisAngularOffset (calibrate)
- [ ] DriveConstants.kBackRightChassisAngularOffset (calibrate)
- [ ] DriveConstants CAN IDs (if wiring changes)
- [ ] VisionConstants.kCameraToRobotX/Y/Z (measure camera position)
- [ ] VisionConstants.kCameraPitch/Yaw/Roll (measure camera orientation)

### Reset Tuning Values (DO NOT CARRY OVER!)
- [ ] TeleopConstants.kHeadingLockP/I/D = 5.0/0.0/0.0 (re-tune)
- [ ] TeleopConstants.kAutoAimP/I/D = 5.0/0.0/0.0 (re-tune)
- [ ] TeleopConstants.kSnapToAngleP/I/D = 5.0/0.0/0.0 (re-tune)
- [ ] AutoConstants.kTranslationPID = (5.0, 0.0, 0.0) (re-tune)
- [ ] AutoConstants.kRotationPID = (5.0, 0.0, 0.0) (re-tune)

### Remove Old Game Code
- [ ] Delete old game-specific subsystems (e.g., AlgaeIntakeSubsystem)
- [ ] Delete old game-specific commands folder (e.g., commands/reefscape/)
- [ ] Clean up RobotContainer button bindings
- [ ] Remove old autonomous routines

---

## 👥 Student Readiness

### Onboarding Materials Ready
- [x] Freshman learning path defined
- [x] Veteran fast-track defined
- [x] Common questions documented
- [x] Quiz yourself sections for self-assessment
- [x] Clear "what NOT to touch" guidance

### Learning Resources
- [x] Links to WPILib docs
- [x] Links to third-party library docs (PathPlanner, PhotonVision)
- [x] Links to video tutorials
- [x] Chief Delphi resources

### Mentorship Support
- [x] Mentor tips in onboarding guide
- [x] Teaching points in technical guides
- [x] Pair programming suggested
- [x] Code review process described

---

## 🔧 Technical Setup

### Repository Structure
- [x] All documentation in docs/ folder
- [x] Year-to-year reusable code identified
- [x] Game-specific code separated
- [x] Clean file organization

### Git Configuration
- [x] .gitignore configured correctly
- [x] No build artifacts committed
- [x] No IDE-specific files committed
- [x] Clean commit history

### FRC Tools Integration
- [ ] Code deploys to robot (test with actual hardware)
- [ ] PathPlanner paths load correctly
- [ ] Dashboard values display correctly
- [ ] Camera streams work

---

## 🧪 Testing Workflow Ready

### PID Tuning
- [x] Tunable commands available (driveWithHeadingLockTunable)
- [x] Dashboard integration explained
- [x] Worksheets provided in guide
- [x] Default starting values documented (5.0, 0.0, 0.0)

### Vision Testing
- [x] Pre-flight checks in guide
- [x] Phase-by-phase procedure
- [x] Safety warnings throughout
- [x] Troubleshooting section
- [x] Success criteria defined

### Button Mappings
- [x] All buttons documented in code comments
- [x] Button reference chart in guide
- [x] Controller ports documented (0, 1, 2)
- [x] Emergency override clearly marked

---

## 📊 Dashboard Configuration

### Required Dashboard Values
- [x] Drive/FieldRelative
- [x] Drive/SpeedMultiplier
- [x] Drive/Heading
- [x] Drive/PoseDrift
- [x] Drive/X_Fused and Drive/Y_Fused
- [x] Drive/FL_Velocity, FR, RL, RR
- [x] Vision/photonvision/Connected
- [x] Vision/photonvision/HasTargets
- [x] Drive/BatteryVoltage
- [x] Drive/BrownoutRisk

### Test Target Configuration
- [x] Test/TargetX
- [x] Test/TargetY
- [x] Test/TargetHeading
- [x] Test/DistanceFromTag

---

## 🎓 Mentor Prep

### Before Handing to Students
- [ ] Read all documentation yourself
- [ ] Test PID tuning workflow on real robot
- [ ] Test vision workflow on real robot
- [ ] Verify all guides make sense
- [ ] Prepare to answer common questions

### Teaching Plan
- [ ] Schedule for introducing students to codebase
- [ ] Pair assignments (veteran + freshman)
- [ ] Weekly goals defined
- [ ] Git workflow explained
- [ ] Code review process established

---

## ✅ Final Sign-Off

**Before declaring "ready for students":**

- [x] All code quality checks pass
- [x] All documentation complete
- [x] All cross-references work
- [x] Freshman and veteran paths clear
- [x] Safety features implemented
- [x] Testing workflows documented
- [x] Year-to-year update checklist created

**Optional (recommended):**
- [ ] Tested on real robot hardware
- [ ] PathPlanner autos created and tested
- [ ] Vision system calibrated
- [ ] PID values tuned for this year's robot
- [ ] Competition-ready autonomous routine

---

## 🚀 Handoff to Students

**What to tell students:**

> "All documentation is in the `docs/` folder. Start with the **Student Onboarding Guide** to learn the codebase. Then follow the **Start of Season Guide** for the complete workflow. We'll do PID tuning together before testing vision."

**What to tell veterans:**

> "This codebase is year-to-year reusable. Don't touch DriveSubsystem or DriveCommands - those are the reusable core. Your job is to tune PID, verify vision, and create game-specific features. See the **Student Onboarding Guide** veteran section for your fast-track."

**What to tell freshmen:**

> "FRC programming is complex - don't worry if it's overwhelming at first! Read the **Student Onboarding Guide** and follow the week-by-week plan. Ask lots of questions. Pair up with a veteran student. You'll get it!"

---

**Use this checklist at the start of EVERY season to ensure smooth student onboarding!**

---

**📚 [Back to Documentation Index](README.md)**

**Last Updated:** 2026-01-16
**Created:** 2026-01-16
