# Vision System Complete Guide

**Purpose:** Complete guide for vision testing - from first-time classroom setup to advanced tuning and competition readiness.

**Choose Your Path:**
- 🆕 **[Beginners → Part 1: Quickstart](#part-1-quickstart-path)** - First-time vision testing (1-2 hours)
- 🎓 **[Advanced → Part 2: Systematic Testing](#part-2-advanced-systematic-testing)** - Detailed procedures and tuning (2-3 days)

**Prerequisites:**
- ✅ Robot drives with joystick controls
- ✅ PhotonVision installed on Raspberry Pi 5
- ✅ Camera connected and configured
- ⚠️ **For Advanced Path:** PID tuning must be complete ([PID_TUNING_GUIDE.md](PID_TUNING_GUIDE.md))

---

## 🧭 Quick Navigation

| I want to... | Go to... |
|--------------|----------|
| Test vision for the first time in classroom | [Part 1: Quickstart](#part-1-quickstart-path) |
| Follow the 8-step testing workflow | [Part 1: Quickstart](#part-1-quickstart-path) |
| Learn about Classroom Mode vs Competition Mode | [Classroom Mode Setup](#step-5-test-auto-rotate-circle-button) |
| Systematically test all vision commands | [Part 2: Advanced Testing](#part-2-advanced-systematic-testing) |
| Tune vision PID values | [Part 2: Phase 5 - Advanced Tuning](#phase-5-advanced-tuning-optional) |
| Troubleshoot vision issues | [Troubleshooting](#troubleshooting) |
| See controller button reference | [Controller Reference](#controller-reference) |

---

# Part 1: Quickstart Path

**Target Audience:** Students testing vision for the first time with 1-3 AprilTags in a classroom

**Time Required:** 1-2 hours (first time), 15 minutes (subsequent sessions)

**What You'll Learn:**
- ✅ Basic robot driving with controllers
- ✅ PhotonVision connection and AprilTag detection
- ✅ Vision-based pose estimation
- ✅ Auto-aim and heading lock commands
- ✅ PathPlanner autonomous movement

---

## The 8-Step Vision Testing Workflow

This quickstart follows your complete testing workflow:

1. ✅ Connect robot to PhotonVision
2. ✅ Camera sees a single AprilTag
3. ✅ Correctly identify which AprilTag
4. ✅ Correctly determine distance to tag
5. ✅ Automatically rotate to face target
6. ✅ Drive while locked facing the target
7. ✅ Drive around classroom with updating pose estimate
8. ✅ One-button command to drive to 1m in front of target

---

## 📋 What You Need

### Hardware
- [ ] Robot with swerve drive and NavX gyro
- [ ] Raspberry Pi 5 with PhotonVision installed
- [ ] Camera connected to Raspberry Pi
- [ ] 2x Thrustmaster joysticks (driver - ports 0 & 1)
- [ ] 1x PlayStation 5 controller (co-driver - port 2)
- [ ] 1-3 printed AprilTags (from [Current Season] field)
- [ ] Fully charged battery (> 12V)
- [ ] Clear space: 3m x 3m minimum

### Software
- [ ] WPILib [Current Year] installed on driver station laptop
- [ ] PhotonVision running on Raspberry Pi
- [ ] Robot code deployed to robot
- [ ] Shuffleboard or Glass dashboard ready

### Recommended AprilTags for Classroom
- **Tag ID 3** - Easy to remember, commonly used in examples
- **Tag ID 7** - Add a second tag for multi-tag testing later
- Print at **6-8 inches** size for optimal detection at 1-4 meters

**Print tags here:** https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings.pdf

---

## Step 0: Pre-Flight Checks (5 minutes)

### Verify Robot is Ready

1. **Battery Check:**
   ```
   Shuffleboard → Drive Tab → Battery Voltage > 12.0V
   ```
   If < 12V, charge or swap battery!

2. **Controllers Connected:**
   ```
   Driver Station → USB Devices
   Should see:
   - Joystick[0] - T.16000M
   - Joystick[1] - T.16000M
   - Joystick[2] - Wireless Controller (PS5)
   ```

3. **Robot Code Deployed:**
   ```
   Driver Station → Status: "Robot Code: READY"
   ```

4. **AprilTag Mounted:**
   - Tape tag to wall at robot height (0.5-1.0m high)
   - Tag should be flat, well-lit, no glare
   - Avoid windows (backlighting confuses camera)

---

## Step 1: Connect to PhotonVision (10 minutes)

### 1.1 Access PhotonVision Dashboard

**Method 1 - mDNS (easiest):**
```
Open browser → http://photonvision.local:5800
```

**Method 2 - Direct IP:**
1. Find Raspberry Pi IP on robot network:
   ```
   Robot network: 10.TE.AM.11 (e.g., 10.56.84.11 for Team 5684)
   ```
2. Open browser → `http://10.TE.AM.11:5800`

**Success:** You should see PhotonVision web interface

**Troubleshooting:**
- ❌ Can't connect → Check Raspberry Pi power LED
- ❌ "Connection refused" → PhotonVision service may not be running
- ❌ Wrong IP → Check robot radio configuration

### 1.2 Configure Camera Name

**CRITICAL:** Camera name in PhotonVision **MUST** match code!

1. In PhotonVision dashboard, click **Settings** tab
2. Find **Camera Nickname** field
3. Set to: **`front`** (lowercase, no spaces!)
4. Click **Save**

**Why?** Code expects `VisionConstants.kFrontCameraName = "front"`

### 1.3 Verify Camera is Detecting Tags

1. Point camera at AprilTag
2. PhotonVision should show:
   - Green box around tag
   - Tag ID number displayed
   - "Targets: 1" in status bar

**Troubleshooting:**
- ❌ No detection → Check lighting (add more light)
- ❌ Wrong pipeline → Select "AprilTag" pipeline
- ❌ Partial detection → Tag too small/far, move closer (1-3m optimal)

### 1.4 Check Connection in Shuffleboard

Enable robot and check:
```
Shuffleboard → Vision Tab
- Vision/front/Connected = ✅ true
- Vision/front/TargetCount = 1 (when pointing at tag)
```

**✅ Step 1 Complete!** Robot is connected to PhotonVision.

---

## Step 2: Basic Driving Test (5 minutes)

**Goal:** Verify robot drives normally before testing vision

### 2.1 Enable Robot

1. Set Driver Station to **TeleOperated** mode
2. Click **Enable**
3. Robot should be **drivable**

### 2.2 Test Basic Driving

**Use driver's left joystick:**
- Push forward → Robot moves forward
- Push left → Robot strafes left
- Push right → Robot strafes right

**Use driver's right joystick (X-axis only):**
- Push left → Robot rotates counterclockwise
- Push right → Robot rotates clockwise

**✅ Success:** Robot responds smoothly to all inputs

**❌ Problem:** Robot doesn't move or moves erratically
- Check battery voltage
- Verify CAN IDs in HardwareConstants.java
- Check motor controller LEDs (should be green)

### 2.3 Test Field-Relative Toggle

1. **Face robot AWAY from driver station**
2. Press **right joystick Button 2** to zero heading
3. Press **left joystick Button 1** to toggle field-relative
4. Check dashboard: `Drive/FieldRelative` changes (true/false)

**Field-Relative ON (true):**
- Forward on joystick → Robot drives away from driver station
- Works regardless of robot rotation

**Field-Relative OFF (false):**
- Forward on joystick → Robot drives toward front of robot
- Relative to robot's orientation

**✅ Step 2 Complete!** Robot drives correctly.

---

## Step 3: Verify AprilTag Detection (5 minutes)

**Goal:** Confirm vision system identifies tags correctly

### 3.1 Point Robot at AprilTag

1. Drive robot to face AprilTag
2. Distance: **2-3 meters** (optimal range)
3. Robot should be roughly centered on tag

### 3.2 Check Tag Detection in Shuffleboard

```
Shuffleboard → Vision Tab → Front Camera Section

Expected:
- Vision/front/Connected = ✅ true
- Vision/front/TargetCount = 1
- Vision/front/TagIDs = "ID3(2.5m)" ← Shows tag ID and distance!
- Vision/front/AvgTagDistance = 2.5 ← Distance in meters
- Vision/front/LatencyMs = 30-50 ← Camera lag (lower is better)
```

**✅ Step 3 Complete!** Camera sees and identifies the tag.

**Troubleshooting:**
- ❌ TargetCount = 0 → Camera doesn't see tag
  - Check PhotonVision dashboard (browser)
  - Ensure tag is well-lit
  - Move closer (1-3m)
- ❌ Wrong tag ID → Printed wrong tag, check PDF
- ❌ Distance seems wrong → Camera transform not calibrated (OK for now)

---

## Step 4: Check Distance Measurement (5 minutes)

**Goal:** Verify distance calculation is reasonable

### 4.1 Measure Actual Distance

Use measuring tape or count floor tiles:
- Measure from **robot center** to **AprilTag center**
- Write down: _______ meters

### 4.2 Compare to Vision Estimate

```
Shuffleboard → Vision Tab
- Vision/front/AvgTagDistance = _______ meters
```

**Expected:** Within ±0.5m of actual distance

**Example:**
- Actual distance: 2.5m
- Vision reports: 2.3m
- Difference: 0.2m ✅ Good!

**Why might it be off?**
- Camera transform not calibrated (VisionConstants.java all zeros)
- Camera mounted at angle (need to measure pitch)
- **This is OK for testing!** We'll fix it later.

**✅ Step 4 Complete!** Distance detection is working.

---

## Step 5: Test Auto-Rotate (Circle Button) (10 minutes)

**Goal:** Robot automatically rotates to face the AprilTag

### 5.1 Enable Classroom Mode (IMPORTANT!)

**Why?** Competition Mode has strict safety filters that will block testing when robot is far from expected position.

```
Shuffleboard → Vision Tab
1. Click "Classroom Mode" checkbox to enable it ☑️
2. Verify "Filter Mode" shows: 🏫 CLASSROOM (5m)
```

**What this does:**
- **Competition Mode (1m threshold):** Rejects vision if robot position differs by > 1m
- **Classroom Mode (5m threshold):** Accepts vision if robot position differs by < 5m
- **Why needed:** When you first place robot, it thinks it's at (0, 0) but vision might say (2.5, 1.0) - difference is 2.6m, rejected in Competition Mode!

### 5.2 Initialize Robot Pose (One-Time Setup)

**First time only:** Tell robot where it actually is

1. Point robot at AprilTag (2-3m away)
2. **Press Touchpad button** (co-driver controller)
3. Watch Shuffleboard:
   ```
   Drive Tab:
   - Drive/Pose should snap to new position
   - Drive/PoseDrift should become small (< 0.5m)
   ```

**What happened?** Force Vision Reset bypassed all safety filters and snapped robot pose to match vision estimate.

### 5.3 Test Auto-Aim Command

1. **Drive robot 90° away from tag** (use joysticks)
2. **Hold Circle button (○)** on co-driver controller
3. **Watch robot rotate:**
   - Should smoothly turn to face AprilTag
   - No oscillation or jerking
   - Stops when facing tag

4. **Check status:**
   ```
   Shuffleboard → Vision Tab
   - Vision/Status = "Accepted: Single-Tag [CLASSROOM]"
   ```
   The `[CLASSROOM]` tag confirms relaxed mode is active!

**Release Circle button** → Robot stops rotating

**✅ Success:** Smooth rotation, faces tag accurately

**❌ Problem - Robot oscillates:**
- PID not tuned → See [PID_TUNING_GUIDE.md](PID_TUNING_GUIDE.md)
- Must complete PID tuning before continuing!

**❌ Problem - Vision/Status says "Rejected":**
- Check that Classroom Mode is enabled (should see `[CLASSROOM]` tag)
- Press Touchpad again to reset pose
- Verify camera sees tag (TargetCount = 1)

**✅ Step 5 Complete!** Auto-aim works.

---

## Step 6: Test Heading Lock (Triangle Button) (10 minutes)

**Goal:** Drive normally while robot auto-rotates to face target

### 6.1 Test Heading Lock

1. **Position robot 3m from AprilTag, facing 45° away**
2. **Hold Triangle button (△)** on co-driver controller
3. **Use driver's left joystick to drive forward/sideways:**
   - Robot keeps rotating to face tag
   - You control translation, robot controls rotation
   - Like "turret mode" - always facing target

4. **Release Triangle** → Robot stops auto-rotating

**✅ Success:** Robot drives smoothly while keeping tag in sight

**Use Case:** During match, drive while keeping shooter aimed at goal!

**✅ Step 6 Complete!** Heading lock works.

---

## Step 7: Pose Estimation Accuracy Test (15 minutes)

**Goal:** Verify pose updates correctly as robot moves around classroom

### 7.1 Watch Pose on Field Widget

```
Shuffleboard → Drive Tab
- Field widget shows blue robot on [Current Season] field
- Robot position should update as you drive
```

### 7.2 Drive Around Classroom

1. **Enable robot**
2. **Drive to 4 different positions** (corners of your space)
3. **At each position:**
   - Face AprilTag briefly
   - Watch `Drive/Pose` update
   - Check `Drive/PoseDrift`

### 7.3 Check Pose Drift

```
Shuffleboard → Drive Tab
- Drive/PoseDrift = Distance between:
  - Fused pose (encoders + vision)
  - Pure odometry (encoders only, no vision)
```

**Good:** PoseDrift < 0.5m
- Vision is correcting encoder drift
- System is healthy

**Problem:** PoseDrift > 1.0m
- Possible issues:
  - Camera transform wrong (VisionConstants.java)
  - Seeing reflections as tags
  - Wrong tag detected
  - Encoders drifting excessively

### 7.4 Test Vision Corrections

1. **Drive robot forward 2m WITHOUT vision** (face away from tag)
2. **Note current pose:** Drive/Pose = (X, Y, θ)
3. **Turn to face AprilTag**
4. **Watch pose snap/adjust** to match vision
5. **Check status:**
   ```
   Vision/Status = "Accepted: Single-Tag [CLASSROOM]"
   ```

**Expected:** Pose adjusts smoothly when tag visible, stays stable when tag not visible

**✅ Step 7 Complete!** Pose estimation is working.

---

## Step 8: Drive-to-Distance Command (Cross Button) (10 minutes)

**Goal:** One-button command drives robot to 1m in front of AprilTag

### 8.1 Set Target Distance

```
Shuffleboard → SmartDashboard Tab
- Find: Test/DistanceFromTag
- Set value: 1.0 (meters)
- Press Enter
```

**This sets:** How far from tag to stop (1m = good testing distance)

### 8.2 Test Drive-to-Distance

1. **Position robot 3-4m from AprilTag**
2. **Press Cross button (✕)** on co-driver controller (hold it!)
3. **Watch robot:**
   - Rotates to face tag
   - Drives forward
   - Stops 1m from tag
   - PathPlanner is controlling the path!

4. **Release Cross button** when robot reaches target

**✅ Success:** Robot autonomously drives to 1m from tag

**How it works:**
- `DriveCommands.positionAtDistance()` calculates target position
- PathPlanner creates path on-the-fly
- Vision keeps pose updated during movement
- Robot arrives at precise distance!

**Experiment:** Try different distances
- Set `Test/DistanceFromTag = 0.5` → Robot stops closer
- Set `Test/DistanceFromTag = 2.0` → Robot stops farther

**✅ Step 8 Complete!** Full autonomous vision movement works!

---

## 🎉 Quickstart Complete!

**You've completed all 8 steps!** Your robot can:
- ✅ Connect to PhotonVision
- ✅ Detect and identify AprilTags
- ✅ Measure distance to tags
- ✅ Auto-rotate to face targets
- ✅ Drive while locked on target
- ✅ Track pose with vision + odometry fusion
- ✅ Autonomously drive to precise positions

**⚠️ BEFORE LEAVING:** Switch back to Competition Mode!
```
Shuffleboard → Vision Tab
1. UNCHECK "Classroom Mode" ☐
2. Verify "Filter Mode" shows: 🏆 COMPETITION (1m)
```

**Next Steps:**
- Want more advanced testing? Continue to [Part 2: Advanced Systematic Testing](#part-2-advanced-systematic-testing)
- Ready for autonomous? See [AUTO_TESTING_GUIDE.md](AUTO_TESTING_GUIDE.md)
- Need to tune PID? See [PID_TUNING_GUIDE.md](PID_TUNING_GUIDE.md)

---

# Part 2: Advanced Systematic Testing

**Target Audience:** Teams who have completed basic vision testing and want detailed procedures, tuning, and troubleshooting

**Prerequisites:**
- ✅ PID tuning complete ([PID_TUNING_GUIDE.md](PID_TUNING_GUIDE.md))
- ✅ Robot rotates smoothly to targets without oscillation
- ✅ All PID values tested at multiple speeds
- ✅ Basic vision testing completed (or comfortable with vision concepts)

**Time Required:** 2-3 days for complete systematic testing

---

## Phase 1: Pre-Flight Verification

### Verify PID is Tuned

**CRITICAL:** Vision commands use PID controllers. If PID isn't tuned, vision tests will fail even if vision works!

**Quick PID Verification Test:**

1. **Enable robot** in teleop mode
2. **Set speed to SLOW mode** (Press L1 bumper on co-driver controller)
3. **Drive 3m away from a wall or marked spot**
4. **Test heading lock WITHOUT vision:**
   - Create a test target at a fixed coordinate (e.g., `Pose2d(5.0, 5.0, 0°)`)
   - Hold Triangle button
   - Robot should smoothly rotate to face that coordinate
5. **Check behavior:**
   - ✅ Smooth rotation, no oscillation → PID is tuned, proceed!
   - ❌ Jerky, oscillating, or no rotation → Go tune PID first!

**If robot oscillates:** Complete [PID_TUNING_GUIDE.md](PID_TUNING_GUIDE.md) before continuing.

### Pre-Test Checklist

- [ ] AprilTag mounted on wall (well-lit, not at extreme angle)
- [ ] PhotonVision dashboard accessible (http://photonvision.local:5800)
- [ ] Robot battery > 12V
- [ ] Controllers connected and recognized in Driver Station
- [ ] E-STOP accessible and tested

---

## Phase 2: Vision System Verification

### 2.1 Basic Drive Verification

**Goal:** Verify robot drives correctly before testing vision

1. **Enable robot** (teleop mode)
2. **Drive with joysticks:**
   - Left stick: Translation (forward/back, left/right)
   - Right stick: Rotation
3. **Test field-relative toggle:**
   - Press **left stick trigger** to toggle modes
   - Field-relative: Forward goes away from driver station
   - Robot-relative: Forward goes toward robot's front
   - Dashboard shows: `Drive/FieldRelative` (true/false)
4. **Verify control feels normal** - no unexpected behavior

### 2.2 Vision Connection Verification

**Goal:** Confirm vision system is working before testing autonomous commands

1. **Turn robot to face AprilTag**
   - Tag should be clearly visible to camera
   - Distance: 1-4 meters (optimal range)

2. **Check PhotonVision Dashboard** (http://photonvision.local:5800)
   - Camera should show green boxes around detected tag
   - Tag ID should be displayed
   - Pipeline should be active

3. **Check FRC Dashboard:**
   ```
   Shuffleboard → Vision Tab
   - Vision/Enabled = true
   - Vision/front/Connected = true
   - Vision/front/TargetCount = 1
   - Vision/front/TagIDs = "ID3(2.5m)"
   - Vision/front/EstimatedPose = "(2.5, 1.0, 45.0°)"
   ```

4. **Verify pose is reasonable:**
   - Position should make sense relative to tag location

---

## Phase 3: Force Vision Reset

**Goal:** Sync odometry with vision to enable accurate autonomous testing

**For Classroom Testing:** First enable Classroom Mode:
```
Shuffleboard → Vision Tab → Check "Classroom Mode" ☑️
Verify "Filter Mode" shows: 🏫 CLASSROOM (5m)
```

**Then reset pose:**

1. **Face AprilTag directly** (as square as possible)

2. **Press TOUCHPAD button** (co-driver controller)

3. **Verify success:**
   - Console shows: `✅ VISION RESET: Pose updated from AprilTag detection`
   - Field/Fused widget shows robot at correct position
   - `Drive/PoseDrift` near 0m (should be < 0.1m)
   - `Vision/Status` shows accepted measurements with `[CLASSROOM]` tag

4. **If reset fails** (console shows ❌):
   - Ensure robot is facing tag
   - Move closer to tag (2-3 meters)
   - Check PhotonVision shows green detection boxes
   - Verify tag is well-lit
   - Verify Classroom Mode is enabled (see above)
   - Try again

**⚠️ IMPORTANT:** Do NOT test autonomous commands until vision reset succeeds!

**⚠️ BEFORE COMPETITION:** Switch back to Competition Mode (uncheck "Classroom Mode")

---

## Phase 4: Configure Test Target

**Goal:** Set the target position for autonomous testing

**Option A: Use Current Position (Recommended)**

1. Drive robot to desired stop position (e.g., 1.5m from tag)
2. Face robot toward tag
3. Press **right stick button 2** to zero heading
4. Note position from dashboard (`Drive/X_Fused`, `Drive/Y_Fused`)
5. Update dashboard values:
   - `Test/TargetX` = current X position
   - `Test/TargetY` = current Y position
   - `Test/TargetHeading` = 0 (facing tag)
   - `Test/DistanceFromTag` = 1.0 (default 1 meter)
6. Press **OPTIONS button** (co-driver controller) to load values

**Option B: Calculate from Tag Position**

If you know tag coordinates (X_tag, Y_tag):
```
TargetX = X_tag - (distance × cos(heading))
TargetY = Y_tag - (distance × sin(heading))
```

---

## Phase 5: Autonomous Command Testing

**⚠️ SAFETY: Start with SLOW speed!**

Press **L1 button** (co-driver controller) to set SLOW mode (50% speed) for safe testing.

---

### Test 1: Auto-Aim (Rotation Only)

**What it does:** Robot rotates to face test target (no translation)

1. **Drive robot away from tag** (3-4 meters, any direction)
2. **Hold Circle button** (co-driver)
   - Robot should rotate to face tag
   - Only rotation - no forward/backward movement
3. **Verify:**
   - Robot rotates smoothly toward tag
   - Stops rotating when facing tag (within ~2°)
   - No oscillation or overshooting
4. **Release Circle button** - rotation stops

**If fails:**
- Check `Drive/X_Fused` and `Drive/Y_Fused` make sense
- Verify test target loaded correctly (press OPTIONS button)
- Check `TeleopConstants.kAutoAimP` (default: 5.0)

---

### Test 2: Drive to Distance

**What it does:** Robot autonomously drives to distance from tag and stops

1. **Drive robot 4-5 meters away from tag**
2. **Face robot approximately toward tag** (doesn't need to be perfect)
3. **Hold Cross button** (co-driver)
   - Robot should drive toward tag
   - Robot should stop at configured distance (default: 1.0m)
4. **Verify:**
   - Robot drives smoothly
   - Stops at correct distance from tag
   - No collision with wall/tag
5. **Release Cross button** to stop early if needed

**⚠️ SAFETY:**
- Test in open space first!
- Be ready to release button if robot goes wrong direction
- Driver can press **right stick button 3** for emergency override

**If fails:**
- Verify vision reset was successful (PoseDrift < 0.1m)
- Check test target values on dashboard
- Increase speed if robot is too slow (press R1 = FAST mode)

---

### Test 3: Heading Lock (Drive While Aiming)

**What it does:** Driver controls translation, robot auto-rotates to face tag

1. **Drive robot 3-4 meters away from tag**
2. **Hold Triangle button** (co-driver)
3. **Driver uses left stick to drive around:**
   - Forward/backward
   - Left/right strafe
4. **Verify:**
   - Robot continuously faces tag while driver moves
   - Smooth rotation tracking
   - Driver has full translation control
5. **Release Triangle button** - return to normal driving

**This is the "drive while aiming" behavior - very useful in matches!**

**If fails:**
- Check `TeleopConstants.kHeadingLockP` (default: 5.0)
- Use `driveWithHeadingLockTunable()` to tune PID on dashboard

---

## Phase 6: Advanced Testing (Optional)

Once basic tests pass, try:

### X-Stance Test
- Press **Square button** (co-driver)
- Wheels should form X pattern
- Robot should resist being pushed
- Release Square to return to normal

### Snap to Cardinal Test
- Hold **L2 trigger** (co-driver)
- Drive with left stick
- Robot should snap to nearest 0/90/180/270°
- Useful for quick field alignment

### Snap to Diamond Test
- Hold **R2 trigger** (co-driver)
- Drive with left stick
- Robot should snap to nearest 45/135/225/315°

### Speed Control Test
- **L1 button:** SLOW mode (50% speed - safe for testing)
- **R1 button:** FAST mode (100% speed - when confident)
- Dashboard shows: `Drive/SpeedMultiplier`

---

## Phase 7: Multi-Tag Testing (If Available)

**Goal:** Test with multiple AprilTags for improved accuracy

### Setup

1. **Mount 2-3 AprilTags** on different walls
   - Spacing: 2-4 meters apart
   - Different tag IDs (e.g., 3, 7, 11)
   - All at same height

2. **Position robot** where it can see 2+ tags simultaneously

3. **Check detection:**
   ```
   Shuffleboard → Vision Tab
   - Vision/front/TargetCount = 2 or 3
   - Vision/front/TagIDs = "ID3(2.4m), ID7(3.1m)"
   ```

### Multi-Tag Benefits

When robot sees 2+ tags:
- **Much higher accuracy** - triangulation between tags
- **Never rejected** - multi-tag always trusted (bypasses safety filters)
- **Lower pose drift** - corrections are more precise

**Check status:**
```
Vision/Status = "Accepted: Multi-Tag"
```

No `[CLASSROOM]` tag needed - multi-tag is always trusted!

---

## Phase 8: Advanced Tuning (Optional)

### Tuning Heading Lock PID

**Use tunable command for live tuning:**

1. **Bind tunable command** (in RobotContainer.java):
   ```java
   coDriver.triangle().whileTrue(
       DriveCommands.driveWithHeadingLockTunable(
           m_robotDrive,
           () -> -driverLeftStick.getY(),
           () -> -driverLeftStick.getX(),
           getTestTarget(),
           0.0
       )
   );
   ```

2. **Open Shuffleboard Tuning Tab:**
   ```
   Tuning → HeadingLockP/I/D
   ```

3. **Test different P values:**
   - Start: 5.0 (default)
   - Too sluggish: Increase P (try 7.0, 10.0)
   - Oscillating: Decrease P (try 3.0, 2.0)

4. **Fine-tune I and D:**
   - I: Reduces steady-state error (usually 0.0)
   - D: Dampens oscillation (usually 0.0)

5. **Save final values** to `TeleopConstants.java`

### Tuning Auto-Aim PID

Same process, but use `autoAimAtTarget()` command and tune `kAutoAimP/I/D`

### Tuning Snap-to-Angle PID

Same process, but use snap commands and tune `kSnapToAngleP/I/D`

---

## 📊 Success Criteria

**You know vision testing is complete when:**

- [x] Robot connects to PhotonVision consistently
- [x] AprilTags detected at 1-4 meter range
- [x] Vision reset works (PoseDrift < 0.1m after reset)
- [x] Auto-aim rotates smoothly without oscillation
- [x] Heading lock maintains aim while driving
- [x] Drive-to-distance reaches target position accurately
- [x] Pose drift stays < 0.5m during extended driving
- [x] Multi-tag detection works (if available)
- [x] All students can operate vision commands confidently

**Ready for competition when:**
- [x] Tested at multiple speeds (50%, 100%)
- [x] Tested with battery at different charge levels
- [x] PID values documented in Constants
- [x] Switched back to Competition Mode
- [x] Pre-match checklist created and practiced

---

# Controller Reference

## Driver (Thrustmaster Joysticks)

```
LEFT STICK:
├── Movement: Translation (X/Y)
└── Button 1: Toggle field-relative

RIGHT STICK:
├── Movement: Rotation
├── Button 1: Toggle speed (full/half)
├── Button 2: Zero heading
└── Button 3: 🚨 EMERGENCY OVERRIDE
```

## Co-Driver (PlayStation 5 Controller)

```
FACE BUTTONS (Vision Testing):
├── Triangle (△): Heading lock (drive while aiming)
├── Circle (○): Auto-aim (rotate to tag)
├── Square (□): X-stance (makes X pattern)
└── Cross (✕): Drive to distance from tag

BUMPERS (Speed Control):
├── L1: SLOW mode (50% speed - safe)
└── R1: FAST mode (100% speed - confident)

TRIGGERS (Rotation Snap):
├── L2: Snap to cardinal (0/90/180/270)
└── R2: Snap to diamond (45/135/225/315)

CENTER BUTTONS (Utilities):
├── TOUCHPAD: 🧪 Force vision reset (bypasses safety filters)
└── OPTIONS: Reload test target from dashboard

D-PAD:
└── Reserved for future game-specific commands
```

---

# Troubleshooting

## Vision Connection Issues

| Problem | Solution |
|---------|----------|
| Camera not detected | Check USB connection, restart robot, verify PhotonVision running |
| No targets detected | Check lighting, tag distance (1-4m optimal), tag orientation |
| Vision reset fails | Face tag directly, move closer, check green boxes in PhotonVision |
| Pose jumps/jitters | Camera transform wrong, seeing reflections, or wrong tag detected |

## Vision Rejected Issues

| Problem | Solution |
|---------|----------|
| Vision rejected (CLASSROOM) | Enable Classroom Mode: Shuffleboard → Vision tab → Check "Classroom Mode" |
| Vision rejected (COMPETITION) | Press TOUCHPAD to force reset, or switch to Classroom Mode for testing |
| Single-tag rejection | Check distance difference - may be > threshold, verify robot pose is reasonable |

## Drive Issues

| Problem | Solution |
|---------|----------|
| Robot drives wrong direction | Check test target values, press OPTIONS to reload |
| Robot too fast | Press L1 button for SLOW mode (50% speed) |
| Robot oscillates when aiming | Lower PID P value in TeleopConstants, or redo PID tuning |
| Robot doesn't move | Check battery > 12V, verify motor brake mode |
| Emergency override needed | Press **right stick button 3** (driver controller) |

## Dashboard Values Reference

| Key | Expected | Meaning |
|-----|----------|---------|
| `Vision/front/Connected` | true | Camera connected |
| `Vision/front/TargetCount` | 1+ | AprilTag(s) detected |
| `Vision/ClassroomMode` | true/false | Classroom (relaxed) vs Competition (strict) filtering |
| `Vision/Status` | "Accepted..." | Why vision was accepted/rejected |
| `Drive/FieldRelative` | true/false | Field vs robot relative |
| `Drive/SpeedMultiplier` | 0.5-1.0 | Current speed % (50% or 100%) |
| `Drive/PoseDrift` | < 0.5m | Difference between fused and pure odometry |

---

# Advanced: Camera Transform Calibration

**For accurate pose estimates, measure camera position:**

## Measuring Camera Transform

1. **Find robot center:** Intersection of wheel diagonals
2. **Measure camera position from center:**
   - X: Forward (+) / Backward (-) in meters
   - Y: Left (+) / Right (-) in meters
   - Z: Up (+) / Down (-) in meters
3. **Measure camera angle:**
   - Pitch: Tilt up/down (radians)
   - Common: -0.52 rad = -30° tilt down
   - Yaw: Pan left/right (0 = forward)
4. **Update VisionConstants.java:**
   ```java
   kFrontCameraToRobotX = 0.25;  // 25cm forward
   kFrontCameraToRobotY = 0.0;   // Centered
   kFrontCameraToRobotZ = 0.5;   // 50cm high
   kFrontCameraPitchRadians = -0.52;  // 30° down
   kFrontCameraYawRadians = 0.0;  // Facing forward
   ```
5. **Redeploy code**
6. **Test again** - distances should be more accurate!

## Multi-Camera Setup

**Adding a second camera (back-facing):**

1. **Connect second camera** to Raspberry Pi
2. **Configure in PhotonVision:**
   - Name it `"back"`
   - Create AprilTag pipeline
3. **Update VisionConstants.java:**
   ```java
   kBackCameraToRobotX = -0.25;  // 25cm backward
   kBackCameraToRobotY = 0.0;
   kBackCameraToRobotZ = 0.5;
   kBackCameraPitchRadians = -0.52;
   kBackCameraYawRadians = Math.PI;  // Facing backward (180°)
   ```
4. **Verify in Shuffleboard:**
   ```
   Vision Tab → Back Camera Section
   - Vision/back/Connected = true
   - Vision/back/TargetCount updates
   ```

**Benefits:**
- 360° AprilTag detection
- More frequent vision corrections
- Better pose accuracy during matches

---

# Related Documentation

**Continue learning:**
- [PID_TUNING_GUIDE.md](PID_TUNING_GUIDE.md) - Must complete before vision testing
- [AUTO_TESTING_GUIDE.md](AUTO_TESTING_GUIDE.md) - PathPlanner autonomous routines
- [SHUFFLEBOARD_GUIDE.md](SHUFFLEBOARD_GUIDE.md) - Dashboard configuration
- [START_OF_SEASON_GUIDE.md](START_OF_SEASON_GUIDE.md) - Full season roadmap

**Understand the code:**
- [ONBOARDING_GUIDE.md](ONBOARDING_GUIDE.md) - Learn the codebase architecture

---

**Last Updated:** 2026-01-18
**Season:** [Current Season]
**Authors:** FRC Template Contributors Programming Mentors
