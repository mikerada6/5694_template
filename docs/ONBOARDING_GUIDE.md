# Student Onboarding Guide

**Purpose:** Get new and veteran students up to speed on this codebase quickly.

**Who this is for:**
- 🆕 **Freshmen** - First time programming FRC robots
- 🎓 **Veterans** - Experienced with FRC but new to this codebase

---

## 🆕 For Freshmen (New to FRC Programming)

Welcome! This guide will help you learn FRC programming step-by-step.

### Week 1: Learn the Basics

**Goal:** Understand what FRC programming is and set up your environment

**Tasks:**
1. **Watch:** [FRC Programming Introduction](https://www.youtube.com/results?search_query=frc+programming+introduction)
2. **Install:** WPILib VS Code (your mentor will help)
3. **Read:** WPILib [Getting Started](https://docs.wpilib.org/en/stable/docs/zero-to-robot/introduction.html)
4. **Understand:** What a subsystem is, what a command is

**Key Concepts:**
- **Robot** = Main entry point (don't modify much)
- **Subsystem** = Controls hardware (motors, sensors)
- **Command** = Actions the robot performs
- **RobotContainer** = Connects buttons to commands

---

### Week 2-3: Explore the Codebase

**Goal:** Understand how THIS robot code is organized

**Start here:** Read [`docs/START_OF_SEASON_GUIDE.md`](START_OF_SEASON_GUIDE.md)

**Files to read (in order):**

1. **`RobotContainer.java`** (150 lines)
   - 📍 **What to look for:**
     - How buttons are mapped (line 161-279)
     - What controllers we use (ports 0, 1, 2)
     - Emergency override button (right stick button 3)
   - ❓ **Quiz yourself:**
     - What button toggles field-relative mode?
     - What does the Triangle button do?
     - Where is the co-driver controller plugged in?

2. **`Constants.java`** (1000+ lines - don't read all at once!)
   - 📍 **Focus on these sections:**
     - `DriveConstants` (lines 80-288) - Robot dimensions, CAN IDs
     - `TeleopConstants` (lines 807-972) - PID values you'll tune
   - ❓ **Quiz yourself:**
     - What's the robot's max speed?
     - What's the track width?
     - What's the default heading lock P value?

3. **`DriveSubsystem.java`** (1000+ lines - READ THE COMMENTS!)
   - 📍 **Don't try to understand every line!** Focus on:
     - The big header comment (lines 29-113) - What subsystem does
     - Method names and what they do
     - How vision is integrated
   - ❓ **Quiz yourself:**
     - What's the difference between "Fused" and "Pure Odometry"?
     - What does `setX()` do?
     - Why do we have two pose trackers?

4. **`DriveCommands.java`** (500+ lines)
   - 📍 **Look for patterns:**
     - All methods return `Command`
     - Most use `Commands.run()` or `Commands.runOnce()`
     - PID controllers for smooth rotation
   - ❓ **Quiz yourself:**
     - What's the difference between `autoAimAtTarget()` and `driveWithHeadingLock()`?
     - What does the tunable version do differently?

---

### Week 4+: Hands-On Practice

**Goal:** Actually work with the robot!

**Follow these guides in order:**
1. [**PID Tuning Guide**](PID_TUNING_GUIDE.md) - Learn how PID works by tuning it
2. [**Vision Guide Part 1**](VISION_GUIDE.md#part-1-quickstart-path) - Test vision commands
3. Help veteran students with game-specific features

**Learning Tips:**
- ✅ **Don't be afraid to ask questions!** FRC is complex.
- ✅ **Pair program** with a veteran student - one codes, one reviews
- ✅ **Make mistakes** on the practice robot, not at competition
- ✅ **Document what you learn** - write comments, take notes
- ✅ **Use the worksheets** in the PID Tuning Guide

---

### Common Freshman Questions

**Q: "There's so much code! Do I need to understand all of it?"**

**A:** No! Focus on:
- **RobotContainer** - button mappings
- **Constants** - what values mean
- **Basic command structure** - how commands are built

You don't need to understand swerve kinematics or pose estimation yet!

---

**Q: "What should I NOT touch?"**

**A:** Avoid modifying:
- **DriveSubsystem.java** - Core swerve physics (reusable year-to-year)
- **MAXSwerveModule.java** - Hardware abstraction
- **ModuleConstants** and **NeoMotorConstants** - Hardware specs

**You CAN modify:**
- **RobotContainer.java** - Button mappings
- **TeleopConstants** - PID tuning values (after tuning!)
- **FieldConstants** - Game-specific positions

---

**Q: "I want to add a new feature. Where do I start?"**

**A:** Three-step process:
1. **Create subsystem** (if needed) - e.g., `IntakeSubsystem.java`
2. **Create commands** - e.g., `runIntake()`, `stopIntake()`
3. **Bind to buttons** in RobotContainer

Ask a mentor or veteran student to help with the first one!

---

**Q: "What if I break something?"**

**A:** Git saves everything! Use:
```bash
git status          # See what you changed
git diff            # See exact changes
git restore FILE    # Undo changes to FILE
```

Your mentor can help recover any mistakes.

---

## 🎓 For Veterans (Experienced with FRC)

Welcome! If you've programmed FRC before, here's your fast-track guide.

### 5-Minute Orientation

**Architecture:** Command-based, WPILib 2025

**Key Features:**
- ✅ MAXSwerve drivetrain (REV Robotics)
- ✅ PhotonVision AprilTag pose estimation
- ✅ Dual pose tracking (vision-fused + pure odometry)
- ✅ PathPlanner integration for autos
- ✅ Year-to-year reusable core

**What's Different from Your Previous Team:**
- **Vision fusion** is built-in (auto-corrects odometry drift)
- **Comprehensive documentation** (guides for everything)
- **PID tuning workflow** (use tunable commands + worksheets)
- **Clean separation:** Reusable core vs game-specific code

---

### Quick Start Checklist

**☐ Read these files first:**
1. [`START_OF_SEASON_GUIDE.md`](START_OF_SEASON_GUIDE.md) - Master workflow
2. `RobotContainer.java` - Button mappings
3. `Constants.java` - All tunable values
4. `DriveCommands.java` - Command factory pattern

**☐ Understand the architecture:**
- **DriveSubsystem** = Physics and state (getters, setters, periodic)
- **DriveCommands** = Behavior factories (return Command objects)
- **VisionSubsystem** = Pose estimation (implements VisionProvider interface)
- **RobotContainer** = Bindings and initialization

**☐ Know what's reusable:**
- ✅ **Reuse every year:** DriveSubsystem, DriveCommands, VisionSubsystem
- 🔄 **Update each year:** RobotContainer, FieldConstants, camera positions
- 🆕 **Add each year:** Game-specific subsystems and commands

---

### Your First Tasks

#### Task 1: Tune PID (Week 1-2)

**Why you:** Veterans understand PID, freshmen are still learning

**Follow:** [PID_TUNING_GUIDE.md](PID_TUNING_GUIDE.md)

**What to tune:**
1. Heading lock PID (most important)
2. Auto-aim PID
3. Snap-to-angle PID
4. PathPlanner PIDs (for autos)

**Time estimate:** 3-5 days

**Deliverable:** Tuned PID values in Constants.java, committed to git

---

#### Task 2: Verify Vision (Week 2-3)

**Follow:** [VISION_GUIDE.md Part 2](VISION_GUIDE.md#part-2-advanced-systematic-testing)

**What to test:**
1. Camera connection and AprilTag detection
2. Force vision reset
3. Auto-aim, drive to distance, heading lock

**Time estimate:** 2-3 days

**Deliverable:** Verified vision system works, documented any issues

---

#### Task 3: Create Game-Specific Features (Week 3-4)

**Examples for 2025 Reefscape:**
- Algae intake subsystem
- Coral shooter subsystem
- Climber subsystem
- Auto routines for scoring

**Pattern to follow:**
```
src/main/java/frc/robot/
├── subsystems/
│   ├── DriveSubsystem.java       ← Reusable (don't touch)
│   ├── VisionSubsystem.java      ← Reusable (don't touch)
│   ├── AlgaeIntakeSubsystem.java ← Game-specific (create new)
│   └── CoralShooterSubsystem.java ← Game-specific (create new)
│
└── commands/
    ├── DriveCommands.java         ← Reusable (don't touch)
    └── reefscape/                 ← Game-specific folder
        ├── IntakeCommands.java    ← Create new
        └── ScoringCommands.java   ← Create new
```

---

#### Task 4: Mentor Freshmen

**Why you:** They'll learn faster with 1-on-1 help

**How to help:**
- **Pair program:** You drive, they navigate (or vice versa)
- **Code review:** Look at their pull requests, give feedback
- **Explain concepts:** "Here's what a subsystem is..."
- **Answer questions:** "Why do we use Commands instead of just methods?"

**Teaching tip:** Don't just give answers - ask guiding questions:
- ❌ "Change line 45 to this..."
- ✅ "What do you think this line does? Let's test your theory."

---

### Advanced Topics for Veterans

#### 1. PathPlanner Autonomous

**Resources:**
- [PathPlanner docs](https://pathplanner.dev/)
- Example usage in `DriveCommands.driveToPose()`
- PID tuning in `AutoConstants`

**Workflow:**
1. Install PathPlanner GUI
2. Load 2025 Reefscape field
3. Draw paths
4. Export to `deploy/pathplanner/paths/`
5. Create auto in `RobotContainer.createAuto()`

---

#### 2. Vision Debugging

**Tools:**
- PhotonVision dashboard: http://photonvision.local:5800
- AdvantageScope for pose visualization
- Dashboard values: `Vision/photonvision/`, `Drive/PoseDrift`

**Common issues:**
- **Pose jumps:** Increase `DriveConstants.kVisionStdDevX/Y/Theta`
- **No targets detected:** Check lighting, camera calibration
- **Wrong position:** Verify camera Transform3d in RobotContainer

---

#### 3. Custom Commands

**Pattern:**
```java
public static Command yourCommand(DriveSubsystem drive, params...) {
    return Commands.run(() -> {
        // Your logic here
        drive.drive(x, y, rot, fieldRelative);
    }, drive).withName("YourCommand");
}
```

**Best practices:**
- Use `Commands.run()` for continuous actions
- Use `Commands.runOnce()` for instant actions
- Always specify requirements with second parameter
- Name your commands with `.withName()`

---

#### 4. Adding New Subsystems

**Checklist:**
1. ☐ Extend `SubsystemBase`
2. ☐ Add hardware initialization in constructor
3. ☐ Implement `periodic()` for telemetry
4. ☐ Add getters/setters for state
5. ☐ Create command factory class (optional but recommended)
6. ☐ Add to RobotContainer
7. ☐ Bind to buttons

**Example:**
```java
public class YourSubsystem extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(CAN_ID, ...);

    public YourSubsystem() {
        // Hardware config
    }

    @Override
    public void periodic() {
        // Telemetry
        SmartDashboard.putNumber("Your/Value", motor.getEncoder().getPosition());
    }

    public void yourMethod(double speed) {
        motor.set(speed);
    }
}
```

---

## 📚 Additional Resources

### WPILib Documentation
- [Command-Based Programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)
- [PID Control](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html)
- [State Space Control](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/index.html)

### Third-Party Libraries
- [PathPlanner](https://pathplanner.dev/) - Autonomous path planning
- [PhotonVision](https://docs.photonvision.org/) - AprilTag vision processing
- [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope) - Log visualization

### Learning Java
- [Oracle Java Tutorials](https://docs.oracle.com/javase/tutorial/)
- [Java for FRC](https://docs.wpilib.org/en/stable/docs/software/java-for-frc/index.html)

### Chief Delphi
- [Chief Delphi Forums](https://www.chiefdelphi.com/) - Ask questions, get help
- Search before asking - chances are someone had the same question!

---

## 🎯 Success Checklist

### Freshmen

After 4 weeks, you should be able to:
- [ ] Explain what a subsystem and command are
- [ ] Navigate the codebase and find where things are
- [ ] Read and understand button mappings in RobotContainer
- [ ] Modify constants without breaking things
- [ ] Follow the PID tuning guide to tune a controller
- [ ] Test vision commands safely
- [ ] Write a simple command with mentor help

---

### Veterans

After 2 weeks, you should be able to:
- [ ] Tune all PID controllers independently
- [ ] Test and verify vision system
- [ ] Create new subsystems following the pattern
- [ ] Write command factories
- [ ] Review freshman code
- [ ] Debug vision issues
- [ ] Create PathPlanner autonomous routines
- [ ] Mentor freshmen effectively

---

## 💡 Tips for All Students

1. **Git is your friend:** Commit often, write good messages
2. **Test incrementally:** Don't add 5 features and then test
3. **Document as you go:** Future you will thank present you
4. **Ask questions:** No question is too simple
5. **Help each other:** Best way to learn is to teach
6. **Read error messages:** They usually tell you what's wrong
7. **Use the dashboard:** SmartDashboard is your debugging tool
8. **Start at 25% speed:** Always test new code slowly first
9. **Print statements work:** `System.out.println()` is debugging
10. **Have fun:** FRC is hard, but it's rewarding!

---

**Welcome to the team! Let's build a great robot! 🤖**

---

**📚 [Back to Documentation Index](README.md)**

**Last Updated:** 2026-01-16
**Game:** 2025 Reefscape
**Team:** [Your Team Number]
