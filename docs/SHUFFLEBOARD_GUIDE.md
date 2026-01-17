# Shuffleboard Dashboard Guide

**Quick Start:** Open Shuffleboard from the FRC Driver Station and it will automatically connect to the robot!

---

## 📊 Dashboard Layout

Your robot is configured with **4 organized tabs** in Shuffleboard:

### 🏆 1. Competition Tab (Use During Matches)
**What:** Minimal, essential info for drivers during competition
- Battery voltage (critical!)
- Field-relative mode status
- Current speed percentage
- Vision system status
- Auto mode selector
- Compact field visualization

**When to use:** During actual competition matches

---

### 🚗 2. Drive Tab (Main Telemetry)
**What:** Complete robot state for monitoring
- Robot X, Y position (meters)
- Heading (degrees)
- Field-relative mode toggle
- Speed percentage
- Battery voltage
- Gyro rotation rate
- Field widget showing robot pose

**When to use:** Practice, testing, and debugging drive issues

---

### 📷 3. Vision Tab (AprilTag Tracking)
**What:** Camera and vision system status
- Vision enabled/disabled toggle
- Vision status indicator:
  - ✅ TRACKING X TAG(S) - Working correctly!
  - ⚠️ CAMERA OFFLINE - Check camera connection
  - 🔍 NO TARGETS - Camera works, but no AprilTags visible
  - ❌ DISABLED - Vision turned off via toggle
- Camera latency (milliseconds)

**When to use:** Testing vision, troubleshooting AprilTag detection

**Tip:** If vision isn't working, check this tab first!

---

### ⚙️ 4. Tuning Tab (For PID Tuning)
**What:** Live PID tuning and debugging telemetry
- **Heading Lock PID values** (kP, kI, kD) - Editable in real-time!
- Heading error (degrees)
- Rotation speed output
- Pose drift (meters) - Difference between vision and pure odometry
- Vision status messages
- Individual module velocities (FL, FR, RL, RR)

**When to use:**
- Following the PID Tuning Guide
- Debugging drive issues
- Checking if swerve modules are responding correctly

**How to tune:**
1. Open Tuning tab
2. Modify kP/kI/kD values directly in Shuffleboard
3. Test robot response immediately
4. Once satisfied, update `DriveConstants.java` with final values

---

### 🎮 5. Commands Tab (Utility Buttons)
**What:** Clickable buttons for common tasks
- 🔄 Force Vision Reset - Snap odometry to AprilTag position
- 🧭 Zero Heading - Reset gyro to 0°
- 🌍 Toggle Field-Relative - Switch driving modes
- 🐇 Full Speed (100%)
- 🏃 Half Speed (50%)
- 🐢 Quarter Speed (25%)
- ❌ X-Stance - Defensive wheel position (wheels form X)
- ↑ Straight Ahead - Align all wheels forward (pre-match setup)
- ⚙️ Coast Mode - Allow robot to be pushed (when disabled)
- 🔒 Brake Mode - Normal driving mode (resists movement)

**When to use:** Testing, troubleshooting, quick adjustments without controller

**Tip:** Mentors can click these buttons from the sidelines during practice!

**Pre-Match Checklist:**
1. Click "↑ Straight Ahead" - aligns wheels forward for easy pushing
2. Click "⚙️ Coast Mode" - makes robot easier to move in pit
3. Before queueing, click "🔒 Brake Mode" - restore normal driving mode

---

## 🎯 Quick Troubleshooting

### "I don't see any data in Shuffleboard!"
1. Check robot is powered on
2. Check you're connected to robot WiFi or USB
3. Look at the top of Shuffleboard - should say "Robot Connected"

### "Vision shows CAMERA OFFLINE"
1. Check USB connection to PhotonVision camera
2. Open PhotonVision dashboard (http://photonvision.local:5800)
3. Verify camera appears in PhotonVision
4. Check camera name matches "photonvision" in code

### "PID values aren't responding when I change them"
1. Make sure you're using the **tunable** heading lock command
2. In `DriveCommands.driveWithHeadingLockTunable()` (not the regular version)
3. Values update every loop cycle

### "Robot pose looks wrong on field widget"
1. Check "Pose Drift" value in Tuning tab
2. If > 1 meter, you need a vision reset
3. Press CREATE button on PlayStation controller
4. Or click "🔄 Force Vision Reset" in Commands tab

---

## 📝 Best Practices

**For Students:**
- Always check Competition tab before matches
- Battery below 12V? Change battery NOW
- Use Tuning tab when following PID Tuning Guide
- Commands tab is your friend when testing alone

**For Mentors:**
- Monitor Drive tab during practice
- Watch for high Pose Drift values
- Use Commands tab to help students remotely
- Competition tab should be the default view during events

**For Drivers:**
- During competition, only look at Competition tab
- Battery voltage is most critical number
- If vision says "OFFLINE", drive without vision assistance
- Speed percentage shows current drive speed limit

---

## 🔧 Advanced: Adding Your Own Widgets

Want to add more telemetry? Two options:

### Option 1: Simple (Auto-layout)
Just add `SmartDashboard.putNumber("MyKey", value)` in your code - it will automatically appear in Shuffleboard!

### Option 2: Organized (Manual layout)
Edit `RobotContainer.configureShuffleboard()` to add widgets with specific positions:
```java
myTab.addNumber("My Value", () -> subsystem.getValue())
    .withPosition(0, 0)  // Column, Row
    .withSize(2, 1);     // Width, Height
```

---

## 📚 Additional Resources

- **AdvantageScope:** Use for post-match analysis and detailed logging
- **PhotonVision Dashboard:** http://photonvision.local:5800 for camera config
- **WPILib Shuffleboard Docs:** https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/index.html

---

**Last Updated:** 2026-01-16
**Game:** 2025 Reefscape
**Team:** 5684
