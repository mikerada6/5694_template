# Autonomous Testing Guide

**Purpose:** Safe procedures for testing PathPlanner autonomous routines.

**Prerequisites:**
- ✅ PID tuning complete (heading lock, auto-aim, PathPlanner PIDs)
- ✅ Vision system tested and verified
- ✅ Robot can drive reliably in teleop

---

## 🧪 Test Autos Included

Two simple, safe test autonomous routines are pre-configured:

### 1️⃣ Test: Drive Forward
**What it does:** Drives straight forward 2 meters and stops

**Purpose:** Verify basic path following
- ✅ Encoders working correctly
- ✅ Gyro maintaining straight line
- ✅ Translation PID tuned
- ✅ Robot stops at end of path

**Expected behavior:** Robot drives straight forward ~6.5 feet, stops smoothly

---

### 2️⃣ Test: L-Shape
**What it does:** Drives forward 2m, turns 90° left, drives 1.5m, stops

**Purpose:** Verify turning and rotation
- ✅ Rotation PID tuned
- ✅ Path transitions smooth
- ✅ Robot maintains position during turn
- ✅ Robot follows multi-waypoint paths

**Expected behavior:** Robot drives forward, smoothly turns left 90°, continues forward, stops

---

## 📋 Pre-Test Checklist

### Safety Setup
- [ ] **Clear space:** At least 4m x 4m open area (no obstacles)
- [ ] **Battery:** Fully charged (> 12.5V)
- [ ] **Emergency plan:** Driver ready with E-STOP
- [ ] **Spectators:** Everyone behind robot starting position
- [ ] **Floor:** Clear of debris, tape, cables

### Robot Configuration
- [ ] **Code deployed:** Latest version with test autos
- [ ] **Dashboard connected:** Can see "Autonomous Command" selector
- [ ] **Controllers connected:** Driver station shows all controllers
- [ ] **Brake mode:** Motors in brake mode (not coast)

---

## 🧪 Testing Procedure

### Phase 1: Select and Verify Auto

1. **Open Driver Station**
2. **Select autonomous mode** (mode selector)
3. **Open SmartDashboard or Shuffleboard**
4. **Find "Autonomous Command" selector**
5. **Select "🧪 Test: Drive Forward"**
6. **Verify selection** shows in dashboard

**⚠️ Common mistake:** Selecting auto but not seeing it in dashboard = old code deployed

---

### Phase 2: Position Robot

1. **Measure starting position:**
   - 1.5m (5 feet) from a wall or reference point
   - 4.0m (13 feet) from left wall
   - Facing parallel to wall (0° rotation)

2. **Mark the position** with tape:
   - Front of robot
   - Expected end position (3.5m from wall = 2m traveled)

3. **Zero the heading:**
   - Switch to **teleop mode**
   - Press **right stick button 2** to zero heading
   - Verify `Drive/Heading` on dashboard shows ~0°
   - Switch back to **autonomous mode**

**Why mark positions?** Helps verify robot actually drove 2 meters!

---

### Phase 3: Run Test Auto #1 (Drive Forward)

**⚠️ EVERYONE STAND BACK!**

1. **Enable autonomous mode** in Driver Station
2. **Watch console output:**
   - Should print: "🚀 Auto started!"
   - Should print: "✅ Auto complete!" after ~2 seconds
3. **Observe robot:**
   - Drives forward smoothly
   - Stays in straight line (no drift left/right)
   - Stops at ~2m mark you placed
4. **Disable robot**
5. **Measure actual distance traveled**

**Success Criteria:**
- ✅ Robot drove straight (< 10cm drift left/right)
- ✅ Robot stopped within 20cm of 2m mark
- ✅ Motion was smooth (no jerking or sudden stops)
- ✅ Console showed start/end messages

**If failed:**
- ❌ Robot drifted left/right → Check gyro calibration, tune translation PID
- ❌ Robot didn't stop at 2m → Check encoder calibration, wheel diameter
- ❌ Robot jerky → Lower AutoConstants.kTranslationPID.kP
- ❌ No console output → Check named commands registered

---

### Phase 4: Run Test Auto #2 (L-Shape)

**Position robot at same starting position (1.5, 4.0)**

1. **Select "🧪 Test: L-Shape"** in autonomous selector
2. **Verify selection** on dashboard
3. **Zero heading** again (teleop → right stick button 2)
4. **Switch to autonomous mode**
5. **Enable robot**
6. **Observe:**
   - Robot drives forward ~2m
   - Robot smoothly turns left 90°
   - Robot continues forward ~1.5m
   - Robot stops facing left
7. **Disable robot**
8. **Check final position and rotation**

**Success Criteria:**
- ✅ Robot turned left (not right!)
- ✅ Turn was smooth (no spinning in place)
- ✅ Robot maintained position during turn
- ✅ Final heading ~90° (check dashboard)

**If failed:**
- ❌ Robot turned wrong direction → Check rotation PID sign
- ❌ Turn was jerky → Lower AutoConstants.kRotationPID.kP
- ❌ Robot drifted during turn → Tune rotation PID, check gyro
- ❌ Didn't reach final position → Check path constraints

---

## 🎯 Testing Checklist

After both test autos pass:

- [ ] Drive Forward completed successfully
- [ ] L-Shape completed successfully
- [ ] Console shows start/end messages
- [ ] Robot stays within expected path (< 20cm error)
- [ ] Motion is smooth and controlled
- [ ] Dashboard values update correctly
- [ ] No brownout warnings during auto

---

## 📊 Tuning PathPlanner PID

**If test autos are not smooth, tune these values in Constants.java:**

### Translation PID (X/Y Position)

```java
// In AutoConstants:
public static final PIDConstants kTranslationPID = new PIDConstants(5.0, 0.0, 0.0);
```

**Symptoms:**
- Robot overshoots waypoints → **Decrease P** (try 4.0, 3.0)
- Robot undershoots waypoints → **Increase P** (try 6.0, 7.0)
- Robot wobbles along path → **Add D** (try 0.3, 0.5)

---

### Rotation PID (Heading)

```java
// In AutoConstants:
public static final PIDConstants kRotationPID = new PIDConstants(5.0, 0.0, 0.0);
```

**Symptoms:**
- Robot overshoots turns → **Decrease P** (try 4.0, 3.0)
- Robot turns too slowly → **Increase P** (try 6.0, 7.0)
- Robot oscillates during turn → **Add D** (try 0.3, 0.5)

---

## 🛠️ Creating Your Own Autos

**Once test autos pass, create game-specific autos:**

### Step 1: Install PathPlanner

1. Download from https://pathplanner.dev/home.html
2. Open PathPlanner application
3. Select "2025 Reefscape" field layout

### Step 2: Create a Path

1. Click "Create New Path"
2. Name it (e.g., "4 Piece Auto")
3. Click on field to place waypoints
4. Drag control points to adjust curve
5. Set starting rotation and velocity
6. Set ending rotation and velocity
7. Add rotation targets (optional)
8. Save path

### Step 3: Set Constraints

**⚠️ Start conservative, increase later:**
- Max Velocity: 2.0 m/s (safe) → 3.0 m/s (competition)
- Max Acceleration: 1.5 m/s²
- Max Angular Velocity: 360°/s
- Max Angular Acceleration: 360°/s²

### Step 4: Create Auto

1. Click "Create New Auto"
2. Name it (same as path)
3. Drag path into sequence
4. Add named commands (if needed)
5. Set starting pose
6. Save auto

### Step 5: Register in RobotContainer

```java
// In createAuto():
auto.addOption("4 Piece Auto", new PathPlannerAuto("4 Piece Auto"));
```

### Step 6: Test!

**Always test new autos at slow speed first:**
1. Lower path constraints in PathPlanner
2. Test in open space
3. Gradually increase speed
4. Test near game pieces/field elements
5. Practice with drive team

---

## 🐛 Troubleshooting

### "Test autos not found" error

**Problem:** Console shows "⚠️ Test autos not found"

**Solution:**
1. Check `deploy/pathplanner/paths/` contains:
   - `Test Drive Forward.path`
   - `Test L-Shape.path`
2. Check `deploy/pathplanner/autos/` contains:
   - `Test Drive Forward.auto`
   - `Test L-Shape.auto`
3. Redeploy code
4. If still failing, recreate paths in PathPlanner GUI

---

### Robot doesn't move in auto

**Possible causes:**
1. Named commands registered after createAuto() → Move registerNamedCommands() before createAuto()
2. Auto not selected on dashboard → Check SmartDashboard/Shuffleboard
3. Robot not enabled → Check Driver Station shows "Enabled"
4. Path constraints too low → Increase max velocity in PathPlanner

---

### Robot moves wrong distance

**Possible causes:**
1. Wheel diameter wrong in Constants → Measure actual wheel diameter
2. Encoders not calibrated → Check encoder counts on dashboard
3. Field coordinates wrong → Verify X/Y in meters (not feet!)

---

### Robot spins in place during paths

**Possible causes:**
1. Rotation PID too aggressive → Lower kRotationPID.kP
2. Starting rotation doesn't match path → Zero heading before enabling
3. Gyro inverted → Check DriveConstants.kGyroReversed

---

## ✅ Competition Readiness

**Before using autos in competition:**

- [ ] All test autos pass consistently (3/3 runs)
- [ ] PathPlanner PIDs tuned and documented
- [ ] Game-specific autos created and tested
- [ ] Autos tested on practice field
- [ ] Drive team practiced selecting correct auto
- [ ] Backup auto created (simple, reliable)
- [ ] Pre-match auto checklist created

**Pre-Match Auto Checklist:**
1. Verify correct auto selected on dashboard
2. Position robot at starting pose
3. Zero heading facing correct direction
4. Check battery > 12V
5. Verify no obstacles in path
6. Driver ready for teleop transition

---

## 📚 Additional Resources

**PathPlanner Documentation:**
- https://pathplanner.dev/
- GUI tutorial videos
- Example autos from other teams

**WPILib Autonomous:**
- https://docs.wpilib.org/en/stable/docs/software/pathplanning/index.html

**Chief Delphi Discussions:**
- Search "PathPlanner" for community help
- Share your autos, get feedback!

---

**📚 [Back to Documentation Index](README.md)**

**Last Updated:** 2026-01-16
**Game:** 2025 Reefscape
**Team:** [Your Team Number]
