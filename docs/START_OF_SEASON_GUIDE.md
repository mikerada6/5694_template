# Start of Season Setup Guide

**Purpose:** Step-by-step workflow for getting your robot competition-ready at the start of each season.

**Audience:** Students setting up a new robot or starting a new season.

**Timeline:** 2-4 weeks depending on complexity

---

## 🎯 Overview

This guide walks you through the **complete setup and testing workflow** in the correct order. Each step builds on the previous one, so **DO NOT skip ahead**!

### The Big Picture

```
Week 1-2: Hardware & Basic Controls
├─ 1. Hardware Assembly
├─ 2. Basic Drive Testing
└─ 3. PID Tuning ← CRITICAL FOUNDATION

Week 2-3: Vision & Autonomous
├─ 4. Vision Hardware Setup
├─ 5. Vision Testing
└─ 6. Autonomous Path Testing

Week 3-4: Game-Specific Features
├─ 7. Add Game Mechanisms
├─ 8. Integration Testing
└─ 9. Competition Practice
```

---

## ⚠️ CRITICAL: Do Things In Order!

### Why Order Matters

**Common Mistake:**
```
❌ Student jumps straight to vision testing
❌ Vision commands oscillate and fail
❌ Student can't tell if it's vision OR control problem
❌ Hours wasted debugging the wrong thing
```

**Correct Approach:**
```
✅ Student tunes PID first (no vision needed)
✅ Robot rotates smoothly with manual controls
✅ THEN add vision
✅ If vision fails, student knows it's a vision problem (not PID)
```

---

## 📅 Week-by-Week Workflow

## Week 1: Hardware Assembly & Basic Drive

### Step 1: Hardware Assembly (2-3 days)

**Goal:** Robot drives with joystick control

**Checklist:**
- [ ] Swerve modules assembled and mounted
- [ ] CAN IDs configured (see Constants.DriveConstants)
- [ ] NavX gyro connected to roboRIO MXP port
- [ ] Battery charged and connected
- [ ] Radio configured and connected
- [ ] Robot code deployed successfully

**Testing:**
1. Deploy code to robot
2. Enable teleop mode
3. Drive with joysticks:
   - Left stick: Translation
   - Right stick: Rotation
4. Verify all 4 modules respond
5. Check dashboard shows `Drive/FL_Velocity`, etc.

**Success Criteria:**
- ✅ Robot drives in all directions
- ✅ Rotation works smoothly
- ✅ No jerky motion or dead modules
- ✅ Battery voltage > 12V

---

### Step 2: Field-Relative Calibration (1 day)

**Goal:** Zero heading and test field-relative driving

**Procedure:**
1. Place robot facing **away from driver station**
2. Press **right stick button 2** to zero heading
3. Press **left stick button 1** to toggle field-relative
4. Drive around - "forward" should always go away from driver station
5. Toggle again - "forward" should go toward robot's front

**Dashboard Check:**
- `Drive/FieldRelative` = true/false
- `Drive/Heading` = 0° when facing away from driver

**Success Criteria:**
- ✅ Field-relative toggle works
- ✅ Heading reads 0° when zeroed
- ✅ Forward direction correct in both modes

---

## Week 1-2: PID Tuning (THE FOUNDATION!)

### Step 3: PID Tuning (3-5 days)

**Goal:** Tune all rotation PID controllers for smooth motion

**⚠️ THIS MUST BE DONE BEFORE VISION TESTING!**

**Why First?**
- Vision commands (Y/A/X) all use PID controllers
- If PID is bad, vision tests will fail even if vision works
- Tuning PID requires NO vision hardware
- You can tune on blocks or in a small space

**Follow:** [PID Tuning Guide](PID_TUNING_GUIDE.md)

**Tuning Order:**
1. **Heading Lock PID** (3-4 hours)
   - Most important for teleop
   - Driver controls translation, robot auto-rotates
   - Test with manual joystick first (no vision!)

2. **Auto-Aim PID** (2-3 hours)
   - Rotate to face a fixed point
   - Test by aiming at a wall corner or marked spot

3. **Snap-to-Angle PID** (1-2 hours)
   - Snap to 0/90/180/270° angles
   - Useful for field alignment

4. **PathPlanner PID** (2-3 hours)
   - Only if doing autonomous paths
   - Tune after vision is working

**Testing Without Vision:**
```java
// Test heading lock by aiming at a FIXED POINT (not AprilTag)
Pose2d fixedTarget = new Pose2d(5.0, 5.0, Rotation2d.fromDegrees(0));

// Hold Triangle button and drive around
// Robot should rotate to keep facing (5, 5) coordinate
```

**Success Criteria:**
- ✅ Robot rotates smoothly to target angle
- ✅ No oscillation at 25%, 50%, 75%, 100% speeds
- ✅ Reaches target within 2° consistently
- ✅ Motion feels responsive but controlled
- ✅ Final PID values documented in Constants.java

**Common Values After Tuning:**
```java
kHeadingLockP = 5.0 - 8.0
kHeadingLockD = 0.5 - 1.5
kHeadingLockI = 0.0 (usually)
```

---

## Week 2: Vision Setup & Testing

### Step 4: Vision Hardware Setup (1-2 days)

**Goal:** Get camera connected and detecting AprilTags

**Prerequisites:**
- ✅ PID tuning complete
- ✅ Robot drives smoothly

**Checklist:**
- [ ] Camera mounted on robot
- [ ] USB connected to roboRIO
- [ ] PhotonVision installed on coprocessor
- [ ] Camera calibrated in PhotonVision
- [ ] AprilTag pipeline configured
- [ ] Camera position measured (X, Y, Z offsets from robot center)
- [ ] Camera rotation measured (roll, pitch, yaw)

**Verification:**
1. Open PhotonVision dashboard (http://photonvision.local:5800)
2. Point camera at AprilTag
3. Green boxes should appear around tag
4. Tag ID should be displayed

**Success Criteria:**
- ✅ PhotonVision shows camera feed
- ✅ AprilTags detected with green boxes
- ✅ Dashboard shows `Vision/photonvision/Connected` = true

---

### Step 5: Vision Testing (2-3 days)

**Goal:** Test all vision-based autonomous commands

**Prerequisites:**
- ✅ PID tuning complete ← CRITICAL!
- ✅ Vision hardware connected
- ✅ AprilTag mounted on wall

**Follow:** [Vision Testing Guide](VISION_TESTING_GUIDE.md)

**Testing Order:**
1. **Vision Reset** (30 min)
   - Face tag, press CREATE button
   - Verify `Drive/PoseDrift` < 0.1m

2. **Auto-Aim Test** (1 hour)
   - Circle button: Rotate to face tag
   - Should be smooth (PID already tuned!)

3. **Drive to Distance Test** (1-2 hours)
   - Cross button: Drive to 1m from tag
   - Test at 25%, 50%, 75% speeds

4. **Heading Lock Test** (1-2 hours)
   - Triangle button: Drive while facing tag
   - Driver uses left stick, robot auto-rotates

**Success Criteria:**
- ✅ Vision reset succeeds consistently
- ✅ Auto-aim rotates smoothly (no oscillation)
- ✅ Drive to distance stops at correct position
- ✅ Heading lock tracks while driving

**If Vision Tests Fail:**
- Check camera Transform3d in RobotContainer
- Verify field layout loaded (k2025Reefscape)
- Check lighting on AprilTag
- Ensure tag is 1-4m away (optimal range)

---

## Week 3: Autonomous Paths

### Step 6: PathPlanner Setup (2-3 days)

**Goal:** Create and test autonomous paths

**Prerequisites:**
- ✅ PID tuning complete
- ✅ Vision testing complete
- ✅ Robot can pathfind to poses

**Setup:**
1. Install PathPlanner software
2. Load 2025 Reefscape field layout
3. Create simple test paths:
   - Straight line (2m forward)
   - L-shape turn
   - S-curve

**Testing:**
1. Deploy path to robot
2. Run in autonomous mode
3. Verify robot follows path accurately

**PID Tuning (if needed):**
- If robot overshoots waypoints: Lower `kTranslationPID.kP`
- If robot is sluggish: Increase `kTranslationPID.kP`
- If robot wobbles: Add `kTranslationPID.kD`

**Success Criteria:**
- ✅ Robot follows straight paths
- ✅ Robot navigates turns smoothly
- ✅ Robot stops at final waypoint
- ✅ Paths complete within expected time

---

## Week 3-4: Game-Specific Features

### Step 7: Add Game Mechanisms (3-5 days)

**Goal:** Integrate scoring mechanisms (shooter, intake, climber, etc.)

**Process:**
1. Create subsystem for each mechanism
2. Add commands in game-specific folder (e.g., ReefscapeCommands.java)
3. Bind to co-driver buttons
4. Test individually before combining

**Example (2025 Reefscape):**
- Algae intake subsystem
- Coral shooter subsystem
- Climber subsystem

---

### Step 8: Integration Testing (2-3 days)

**Goal:** Test drive + game mechanisms together

**Scenarios:**
1. Drive to scoring position + shoot
2. Intake while driving
3. Drive to climb position + climb

**Watch for:**
- Brownout (battery voltage drops)
- Command conflicts (two commands need same subsystem)
- Timing issues (commands don't sequence correctly)

---

### Step 9: Competition Practice (ongoing)

**Goal:** Practice driving and strategy

**Drills:**
1. Speed runs (complete game tasks quickly)
2. Accuracy tests (score from different positions)
3. Defense practice (push/resist)
4. Recovery practice (what if vision fails?)

---

## 📋 Master Checklist

### Before Competition

**Hardware:**
- [ ] All motors respond correctly
- [ ] Battery holds charge (test for 2+ minute runs)
- [ ] All sensors connected and reading values
- [ ] Camera detects tags reliably
- [ ] Bumpers installed and secure

**Software:**
- [ ] PID values tuned and documented
- [ ] Vision system tested and verified
- [ ] Autonomous paths tested
- [ ] Emergency stop works (right stick button 3)
- [ ] Code committed to git with clear messages

**Testing:**
- [ ] Drive testing complete (field-relative works)
- [ ] Vision testing complete (all phases pass)
- [ ] Autonomous testing complete (paths run)
- [ ] Game-specific testing complete
- [ ] Full practice matches run successfully

**Documentation:**
- [ ] PID values recorded in Constants.java
- [ ] Camera position/rotation documented
- [ ] Button mappings documented
- [ ] Known issues list created
- [ ] Pre-match checklist created

---

## 🎯 Quick Reference: What to Do When

### "We just got our robot hardware!"
→ Start at **Step 1: Hardware Assembly**

### "Robot drives but rotates jerkily"
→ Do **Step 3: PID Tuning** (don't skip this!)

### "We want to test vision"
→ First complete **Step 3: PID Tuning**, THEN **Step 5: Vision Testing**

### "Vision commands oscillate and fail"
→ Go back to **Step 3: PID Tuning** (you skipped it!)

### "We want to create autonomous paths"
→ Complete Steps 1-5 first, then **Step 6: PathPlanner**

### "Competition is next week!"
→ Follow **Step 9: Competition Practice** + use pre-match checklist

---

## 🐛 Common Mistakes & How to Avoid Them

### Mistake 1: Skipping PID Tuning

**What happens:**
```
Student: "Let's test vision!"
*Robot oscillates wildly during auto-aim*
Student: "Vision is broken!"
Mentor: "Did you tune PID first?"
Student: "...what's PID?"
```

**Solution:** Follow this guide in order. PID tuning is Week 1-2, vision is Week 2-3.

---

### Mistake 2: Testing at Full Speed First

**What happens:**
```
Student: "Let's test drive to distance at 100% speed!"
*Robot slams into wall*
```

**Solution:** ALWAYS start at 25% speed. Gradually increase: 25% → 50% → 75% → 100%

---

### Mistake 3: Not Documenting Tuning

**What happens:**
```
Student tunes PID to perfection.
Code gets re-deployed from git (old values).
Student: "It was working yesterday! What changed?"
```

**Solution:** Use the PID Tuning Worksheet. Commit values to git immediately after tuning.

---

### Mistake 4: Testing Without Force Vision Reset

**What happens:**
```
Student: "Drive to distance doesn't work!"
*Odometry is drifted 2m from reality*
Mentor: "Did you press BACK to reset vision?"
Student: "Oh..."
```

**Solution:** Phase 3 of vision testing: ALWAYS reset vision before testing commands.

---

### Mistake 5: Adding Too Many Features at Once

**What happens:**
```
Student adds intake, shooter, climber, and vision in one day.
Nothing works.
No idea which part is broken.
```

**Solution:** Add one feature at a time. Test. Commit. Repeat.

---

## 📚 Guide Quick Links

- **PID Tuning:** [PID Tuning Guide](PID_TUNING_GUIDE.md) (Week 1-2)
- **Vision Testing:** [Vision Testing Guide](VISION_TESTING_GUIDE.md) (Week 2-3)
- **Code Documentation:** See comments in DriveSubsystem.java, DriveCommands.java

---

## 🎓 Teaching Points

**For mentors:**

1. **Enforce the order:** Don't let students skip ahead. Each step builds on previous ones.

2. **Document everything:** Use worksheets, take videos, write down what works.

3. **One change at a time:** Scientific method - change one variable, observe result.

4. **Celebrate milestones:** When PID tuning is done, that's a BIG WIN! Acknowledge it.

5. **Build confidence:** Start simple (25% speed), build up to complex (100% with vision).

**For students:**

1. **Be patient:** PID tuning takes time. It's not "click and done."

2. **Trust the process:** The order exists for a reason. Don't skip steps.

3. **Learn from failures:** If a test fails, that's DATA! Write down what happened.

4. **Ask why:** Don't just copy values. Understand what P, I, D do.

5. **Help each other:** Pair programming works! One drives, one tunes, both learn.

---

## ✅ Final Pre-Competition Checklist

**24 Hours Before Competition:**

- [ ] Test all drive modes (field-relative, robot-relative)
- [ ] Test all vision commands (Triangle, Circle, Cross buttons)
- [ ] Test autonomous paths
- [ ] Verify E-STOP works
- [ ] Check battery voltage > 12.5V
- [ ] Verify all dashboard values display correctly
- [ ] Print this checklist and bring to competition!

**At Competition (Before Each Match):**

- [ ] Zero heading facing away from driver station
- [ ] Check `Vision/photonvision/Connected` = true
- [ ] Test drive forward/back/left/right
- [ ] Verify field-relative works
- [ ] Check battery voltage
- [ ] Review autonomous path for this match

---

**Remember:** You can't build a house starting with the roof. You can't test vision without PID. Follow the order, trust the process, and you'll have a competition-ready robot!

---


---

**📚 [Back to Documentation Index](README.md)**

**Last Updated:** 2026-01-16
**Game:** 2025 Reefscape (update each year!)
**Team:** [Your Team Number]
