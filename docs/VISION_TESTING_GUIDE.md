# Vision Testing Guide

**Purpose:** Safe testing procedures for vision-based autonomous commands with a single AprilTag in the classroom.

**⚠️ CRITICAL PREREQUISITE:** You **MUST** complete PID tuning (see [PID Tuning Guide](PID_TUNING_GUIDE.md)) BEFORE running this guide! Vision commands will fail if PID is not tuned.

**Prerequisites:**
- ✅ PID tuning complete (Heading Lock, Auto-Aim, Snap-to-Angle)
- ✅ Robot rotates smoothly to targets without oscillation
- ✅ All PID values tested at 25%, 50%, 75%, 100% speeds
- ✅ PID values documented in Constants.java

**Hardware Setup:**
- Driver: 2x Thrustmaster T.16000M joysticks (ports 0 & 1)
- Co-Driver: PlayStation controller (port 2)
- Vision: PhotonVision camera configured with AprilTag pipeline

**See Also:** [Start of Season Guide](START_OF_SEASON_GUIDE.md) for complete setup workflow

---

## 🎯 Quick Start

**IF YOU HAVE ALREADY TUNED PID:**
1. Place AprilTag on classroom wall
2. Turn on robot and verify vision connection
3. Face tag and press **CREATE** to force vision reset
4. Test vision commands (Triangle/Cross/Square buttons)

**IF YOU HAVE NOT TUNED PID YET:**
❌ **STOP!** Do not proceed with this guide.
→ Complete [PID Tuning Guide](PID_TUNING_GUIDE.md) first
→ Then come back here

---

## ⚠️ Pre-Flight Check: Is PID Tuned?

**Before starting vision testing, verify PID tuning is complete:**

### Quick PID Verification Test

1. **Enable robot** in teleop mode
2. **Set speed to 25%** (D-Pad Left)
3. **Drive 3m away from a wall or marked spot**
4. **Test heading lock WITHOUT vision:**
   - Create a test target at a fixed coordinate (e.g., `Pose2d(5.0, 5.0, 0°)`)
   - Hold Triangle button
   - Robot should smoothly rotate to face that coordinate
5. **Check behavior:**
   - ✅ Smooth rotation, no oscillation → PID is tuned, proceed!
   - ❌ Jerky, oscillating, or no rotation → Go tune PID first!

**If robot oscillates or fails this test:** Your PID is NOT tuned. Complete [PID Tuning Guide](PID_TUNING_GUIDE.md) before continuing.

---

## 📋 Pre-Test Checklist

- [ ] AprilTag mounted on wall (well-lit, not at extreme angle)
- [ ] PhotonVision dashboard accessible (http://photonvision.local:5800)
- [ ] Robot battery > 12V
- [ ] Controllers connected and recognized in Driver Station
- [ ] E-STOP accessible and tested

---

## 🧪 Testing Procedure

### Phase 1: Basic Drive Verification

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

---

### Phase 2: Vision Verification

**Goal:** Confirm vision system is working before testing autonomous commands

1. **Turn robot to face AprilTag**
   - Tag should be clearly visible to camera
   - Distance: 1-4 meters (optimal range)

2. **Check PhotonVision Dashboard** (http://photonvision.local:5800)
   - Camera should show green boxes around detected tag
   - Tag ID should be displayed
   - Pipeline should be active

3. **Check FRC Dashboard:**
   - `Vision/photonvision/Connected` = **true**
   - `Vision/photonvision/HasTargets` = **true**
   - `Vision/photonvision/TargetCount` = **1** (or more)

4. **Check AdvantageScope** (if available):
   - Vision ghost should show robot position based on AprilTag
   - Position should make sense relative to tag location

---

### Phase 3: Force Vision Reset

**Goal:** Sync odometry with vision to enable accurate autonomous testing

1. **Face AprilTag directly** (as square as possible)

2. **Press CREATE button** (co-driver controller)

3. **Verify success:**
   - Console shows: `✅ VISION RESET: Pose updated from AprilTag detection`
   - Field/Fused widget shows robot at correct position
   - `Drive/PoseDrift` near 0m (should be < 0.1m)

4. **If reset fails** (console shows ❌):
   - Ensure robot is facing tag
   - Move closer to tag (2-3 meters)
   - Check PhotonVision shows green detection boxes
   - Verify tag is well-lit
   - Try again

**⚠️ IMPORTANT:** Do NOT test autonomous commands until vision reset succeeds!

---

### Phase 4: Configure Test Target

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
6. Press **START button** to load values

**Option B: Calculate from Tag Position**

If you know tag coordinates (X_tag, Y_tag):
```
TargetX = X_tag - (distance × cos(heading))
TargetY = Y_tag - (distance × sin(heading))
TargetHeading = heading to face tag
```

---

### Phase 5: Autonomous Command Testing

**⚠️ SAFETY: Start with 25% speed!**

Press **D-Pad LEFT** to set speed to 25% for safe testing.

---

#### Test 1: Auto-Aim (Rotation Only)

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
- Verify test target loaded correctly (press START)
- Check `TeleopConstants.kAutoAimP` (default: 5.0)

---

#### Test 2: Drive to Distance

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
- Increase speed if robot is too slow (D-Pad RIGHT = 50%)

---

#### Test 3: Heading Lock (Drive While Aiming)

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

### Phase 6: Advanced Testing (Optional)

Once basic tests pass, try:

#### X-Stance Test
- Press **Square button** (co-driver)
- Wheels should form X pattern
- Robot should resist being pushed
- Release Square to return to normal

#### Snap to Cardinal Test
- Hold **L1 button** (co-driver)
- Drive with left stick
- Robot should snap to nearest 0/90/180/270°
- Useful for quick field alignment

#### Snap to Diamond Test
- Hold **R1 button** (co-driver)
- Drive with left stick
- Robot should snap to nearest 45/135/225/315°

#### Speed Control Test
- **D-Pad UP:** 100% speed (full)
- **D-Pad RIGHT:** 75% speed
- **D-Pad DOWN:** 50% speed (half)
- **D-Pad LEFT:** 25% speed (precise)
- Dashboard shows: `Drive/SpeedMultiplier`

---

## 🎮 Button Reference

### Driver (Thrustmaster Joysticks)

```
LEFT STICK:
├── Movement: Translation (X/Y)
└── Trigger: Toggle field-relative

RIGHT STICK:
├── Movement: Rotation
├── Button 1: Toggle speed (full/half)
├── Button 2: Zero heading
└── Button 3: 🚨 EMERGENCY OVERRIDE
```

### Co-Driver (PlayStation 5 Controller)

```
FACE BUTTONS:
├── Triangle (△): Heading lock (drive while aiming)
├── Circle (○): Auto-aim (rotate to tag)
├── Square (□): X-stance (makes X pattern)
└── Cross (✕): Drive to distance from tag

BUMPERS:
├── L1: Snap to cardinal (0/90/180/270)
└── R1: Snap to diamond (45/135/225/315)

CENTER BUTTONS:
├── CREATE: 🧪 Force vision reset
└── OPTIONS: Reload test target

D-PAD:
├── UP: 100% speed
├── RIGHT: 75% speed
├── DOWN: 50% speed
└── LEFT: 25% speed
```

---

## 🔧 Troubleshooting

### Vision Issues

| Problem | Solution |
|---------|----------|
| Camera not detected | Check USB connection, restart robot, verify PhotonVision running |
| No targets detected | Check lighting, tag distance (1-4m optimal), tag orientation |
| Vision reset fails | Face tag directly, move closer, check green boxes in PhotonVision |
| Pose jumps/jitters | Increase `DriveConstants.kVisionStdDevX/Y/Theta` to trust vision less |

### Drive Issues

| Problem | Solution |
|---------|----------|
| Robot drives wrong direction | Check test target values, press START to reload |
| Robot too fast | Use D-Pad to reduce speed (LEFT = 25%) |
| Robot oscillates when aiming | Lower PID P value in TeleopConstants |
| Robot doesn't move | Check battery > 12V, verify motor brake mode |
| Emergency override needed | Press **right stick button 3** |

### Dashboard Values

| Key | Expected | Meaning |
|-----|----------|---------|
| `Vision/photonvision/Connected` | true | Camera connected |
| `Vision/photonvision/HasTargets` | true | AprilTag detected |
| `Drive/FieldRelative` | true/false | Field vs robot relative |
| `Drive/SpeedMultiplier` | 0.25-1.0 | Current speed % |
| `Drive/PoseDrift` | < 0.5m | Difference between fused and pure odometry |

---

## 📊 Success Criteria

### Vision System
- ✅ Camera connects to robot
- ✅ AprilTag consistently detected
- ✅ Vision reset succeeds (PoseDrift < 0.1m after reset)
- ✅ Fused pose matches expected position

### Auto-Aim
- ✅ Robot rotates to face tag
- ✅ Stops within 2° of target heading
- ✅ No oscillation or overshooting

### Drive to Distance
- ✅ Robot drives to target position
- ✅ Stops at correct distance (±0.2m)
- ✅ Smooth approach, no jerky motion

### Heading Lock
- ✅ Robot faces tag while driver translates
- ✅ Smooth rotation tracking
- ✅ Driver maintains full translation control

---

## 🎓 Teaching Points

**For students new to vision:**

1. **Why force vision reset?**
   - Odometry drifts over time (wheel slip, carpet friction)
   - Vision provides absolute position from AprilTags
   - Reset "snaps" odometry to vision truth

2. **Why test each command separately?**
   - Isolates problems (is it vision? controls? PID?)
   - Builds confidence in each system
   - Safer than testing everything at once

3. **Why start at 25% speed?**
   - Gives time to react if something goes wrong
   - Easier to observe behavior
   - Reduces risk of damage

4. **What's the difference between auto-aim and heading lock?**
   - Auto-aim: Robot controls everything (rotation + translation)
   - Heading lock: Robot controls rotation, driver controls translation
   - Heading lock is more versatile in matches!

---

## 📝 Notes

- These procedures assume a **single AprilTag** for testing
- Field layout and coordinates will change each game year
- Test in open space before testing near obstacles
- Always have E-STOP accessible
- Document any PID tuning changes in Constants.java

---


---

**📚 [Back to Documentation Index](README.md)**

**Last Updated:** 2026-01-16
**Game:** 2025 Reefscape
**Team:** [Your Team Number]
