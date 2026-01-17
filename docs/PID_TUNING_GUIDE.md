# PID Tuning Guide

**Purpose:** Step-by-step procedures for tuning PID controllers in the drive and vision systems.

**Audience:** Students learning PID control for the first time.

**⚠️ DO THIS FIRST!** PID tuning must be completed **BEFORE** vision testing. See [Start of Season Guide](START_OF_SEASON_GUIDE.md) for complete setup workflow.

**Prerequisites:**
- ✅ Robot drives with joystick control
- ✅ Field-relative toggle works
- ✅ Basic drive testing complete
- ❌ Vision NOT required (tune PID without cameras!)

---

## 📚 What is PID?

**PID = Proportional + Integral + Derivative**

A PID controller automatically adjusts robot behavior to reach a target (position, angle, speed, etc.).

### Simple Analogy: Shower Temperature

Imagine adjusting a shower to the perfect temperature:

- **P (Proportional):** How hard you turn the knob based on how far from perfect the temperature is
  - Too cold → turn hot knob a lot
  - Slightly cold → turn hot knob a little
  - Problem: You might overshoot and make it too hot!

- **D (Derivative):** Slow down as you get closer to the target
  - Temperature rising fast → ease off the knob
  - Prevents overshooting
  - Like applying brakes as you approach a stop sign

- **I (Integral):** Fix persistent errors
  - Been slightly cold for a while → add a little more hot
  - Corrects steady-state errors
  - Usually not needed for FRC rotation control

---

## 🎯 What PID Controllers Do We Have?

This codebase has **5 PID controllers** that need tuning:

| Controller | Purpose | When to Tune |
|------------|---------|--------------|
| **Heading Lock** | Face target while driving | After basic drive works |
| **Auto-Aim** | Rotate to face target | After heading lock works |
| **Snap-to-Angle** | Snap to cardinal/diamond angles | After auto-aim works |
| **PathPlanner Translation** | Follow paths (X/Y movement) | When testing autonomous |
| **PathPlanner Rotation** | Follow paths (rotation) | When testing autonomous |

**Tuning Order:** Start with heading lock, then auto-aim, then snap-to-angle, then PathPlanner.

---

## 🔧 Tuning Workflow Overview

For each PID controller:

1. **Start with defaults** (P=5.0, I=0.0, D=0.0)
2. **Test current behavior** - Observe what happens
3. **Adjust P value** - Tune until "good enough"
4. **Add D if needed** - Only if oscillating
5. **Ignore I** - Almost never needed for rotation
6. **Record final values** in Constants.java
7. **Test again** to verify

---

## 📋 Safety Rules for PID Tuning

⚠️ **ALWAYS follow these rules:**

- [ ] Start at **25% speed** (D-Pad Left)
- [ ] Test in **open space** (at least 3m clearance)
- [ ] Have **E-STOP accessible**
- [ ] **One person** changes values, **one person** drives
- [ ] **Document** what you try (write it down!)
- [ ] **Robot on blocks** for initial tests (wheels off ground)
- [ ] Increase speed gradually: 25% → 50% → 75% → 100%

---

## 🎮 Tuning Method 1: Dashboard Live Tuning

**Best for:** Quick iteration, finding ballpark values

**How it works:** Change PID values on dashboard while robot is enabled

### Setup

1. **Enable tunable command:**
   ```java
   // In RobotContainer, temporarily replace heading lock with tunable version:
   coDriver.x()
       .whileTrue(DriveCommands.driveWithHeadingLockTunable(
           m_robotDrive,
           () -> -driverLeftStick.getY(),
           () -> -driverLeftStick.getX(),
           () -> getTestTarget()
       ));
   ```

2. **Deploy code and open dashboard** (Shuffleboard or Glass)

3. **Find tuning values** on dashboard:
   - `HeadingLock/TunableP`
   - `HeadingLock/TunableI`
   - `HeadingLock/TunableD`

### Tuning Procedure

1. **Set initial values:**
   - P = 5.0
   - I = 0.0
   - D = 0.0

2. **Place robot 3m from AprilTag**

3. **Set speed to 25%** (D-Pad Left)

4. **Hold Triangle button** (heading lock) and observe:

**What to look for:**

| Behavior | Problem | Solution |
|----------|---------|----------|
| Robot doesn't rotate | P too low | Increase P by 1.0 |
| Robot rotates slowly | P too low | Increase P by 1.0 |
| Robot rotates but overshoots and oscillates | P too high | Decrease P by 1.0 |
| Robot oscillates back and forth continuously | P too high, needs damping | Decrease P, add D=0.5 |
| Robot almost reaches target but doesn't finish | P too low at small errors | Increase P slightly |
| Robot is "twitchy" or "jittery" | D too low or P too high | Add D=0.5 or reduce P |

5. **Adjust P value on dashboard** (changes take effect immediately)

6. **Repeat** until robot smoothly rotates to target and stops

7. **If oscillating, add D:**
   - Start with D = 0.5
   - Increase D by 0.1 until oscillation stops
   - Too much D makes system sluggish

8. **Test at higher speeds:**
   - 50% speed (D-Pad Down)
   - 75% speed (D-Pad Right)
   - 100% speed (D-Pad Up)
   - Re-tune if behavior changes significantly

9. **Record final values** and update Constants.java

---

## 🎮 Tuning Method 2: Code + Deploy

**Best for:** Final tuning, precise values

**How it works:** Change values in Constants.java, redeploy, test

### Procedure

1. **Edit Constants.java:**
   ```java
   public static final class TeleopConstants {
       // Heading lock PID constants
       public static final double kHeadingLockP = 5.0;  // ← Change this
       public static final double kHeadingLockI = 0.0;
       public static final double kHeadingLockD = 0.0;  // ← Or this
   ```

2. **Deploy code** (F5 or `./gradlew deploy`)

3. **Test** with Triangle button (heading lock)

4. **Observe behavior** (use table above)

5. **Repeat** until good

**Slower than live tuning, but builds good habits!**

---

## 📊 PID Tuning: Step-by-Step Examples

### Example 1: Tuning Heading Lock PID

**Goal:** Robot faces AprilTag while driver controls translation

**Starting values:** P=5.0, I=0.0, D=0.0

#### Step 1: Test Default Behavior

1. Deploy code with defaults
2. Place robot 3m from tag
3. Set speed to 25%
4. Hold Triangle button
5. Driver moves left stick around

**Observation:** "Robot rotates toward tag but oscillates back and forth, never settling"

**Diagnosis:** P is too high, needs damping

#### Step 2: Add Derivative

1. Set D = 0.5
2. Deploy and test

**Observation:** "Robot rotates smoothly but stops 5° short of target"

**Diagnosis:** P might be too low now, or D is too high

#### Step 3: Increase P Slightly

1. Set P = 6.0, D = 0.5
2. Deploy and test

**Observation:** "Robot rotates smoothly and stops at target! Perfect!"

#### Step 4: Test at Higher Speeds

1. Set speed to 50% (D-Pad Down)
2. Test again

**Observation:** "Still good!"

1. Set speed to 100% (D-Pad Up)
2. Test again

**Observation:** "Slight oscillation at full speed"

#### Step 5: Fine-Tune for Full Speed

1. Set D = 0.7
2. Deploy and test at 100%

**Observation:** "Perfect at all speeds!"

#### Final Values

```java
public static final double kHeadingLockP = 6.0;
public static final double kHeadingLockI = 0.0;
public static final double kHeadingLockD = 0.7;
```

**Document in commit message:**
```
Tuned heading lock PID to P=6.0, D=0.7
- Smooth rotation at all speeds
- No oscillation
- Reaches target within 2 degrees
```

---

### Example 2: Tuning Auto-Aim PID

**Goal:** Robot rotates to face target (no translation)

**Starting values:** P=5.0, I=0.0, D=0.0

#### Common Issue: Too Slow

**Observation:** "Robot rotates toward tag very slowly, takes 5+ seconds"

**Solution:** Increase P to 8.0

**Result:** "Much faster! But now overshoots and oscillates"

**Solution:** Add D=1.0

**Result:** "Perfect! Fast and smooth."

#### Final Values

```java
public static final double kAutoAimP = 8.0;
public static final double kAutoAimI = 0.0;
public static final double kAutoAimD = 1.0;
```

---

### Example 3: PathPlanner PID Tuning

**Goal:** Robot follows autonomous paths accurately

**When to tune:** After heading lock and auto-aim work well

#### Translation PID

Controls how robot follows X/Y path coordinates.

**Test procedure:**
1. Create simple straight-line auto in PathPlanner (2m forward)
2. Run auto and observe

**Common issues:**

| Observation | Solution |
|-------------|----------|
| Robot doesn't move | Increase P (try 7.0) |
| Robot overshoots waypoints | Decrease P or add D |
| Robot "wobbles" along path | Add D=0.5 |
| Robot is slow to correct errors | Increase P |

**Starting recommendation:**
```java
public static final PIDConstants kTranslationPID = new PIDConstants(5.0, 0.0, 0.0);
```

**After tuning (example):**
```java
public static final PIDConstants kTranslationPID = new PIDConstants(7.0, 0.0, 0.5);
```

#### Rotation PID

Controls how robot maintains heading during auto paths.

**Usually same as heading lock PID!**

**If different:**
```java
public static final PIDConstants kRotationPID = new PIDConstants(6.0, 0.0, 0.7);
```

---

## 🔍 Diagnosing PID Problems

### Visual Guide to PID Behavior

#### **Good PID Tuning**
```
Target: ─────────|─────────

Robot:  ↗↗↗↗↗↗↗↗↗╱──────

Time:   0s    1s    2s
```
- Smooth approach
- Reaches target quickly
- No overshoot
- Stays at target

---

#### **P Too Low**
```
Target: ─────────|─────────

Robot:  ─→→→→→→→→↗

Time:   0s    2s    4s
```
- Slow response
- Takes forever to reach target
- Never quite gets there

**Fix:** Increase P

---

#### **P Too High, No D**
```
Target: ─────────|─────────
                 ↗↘↗↘↗↘
Robot:  ↗↗↗↗↗↗↗↗╱

Time:   0s    1s    2s
```
- Fast initial response
- Overshoots target
- Oscillates back and forth
- Never settles

**Fix:** Decrease P OR add D

---

#### **P High with Good D**
```
Target: ─────────|─────────
                ╱╲
Robot:  ↗↗↗↗↗↗↗╱──╲────

Time:   0s    1s    2s
```
- Fast approach
- Small overshoot (acceptable)
- Quickly settles
- Good performance!

**Fix:** This is good! Maybe tweak D slightly if overshoot bothers you.

---

#### **D Too High**
```
Target: ─────────|─────────

Robot:  ───→→→→→↗

Time:   0s    2s    4s
```
- Sluggish response
- Approaches slowly
- Takes long time
- "Feels heavy"

**Fix:** Decrease D

---

## 📝 PID Tuning Worksheet

**Print this out and fill in during tuning!**

### Heading Lock PID

| Test | P Value | I Value | D Value | Speed | Behavior | Notes |
|------|---------|---------|---------|-------|----------|-------|
| 1 | 5.0 | 0.0 | 0.0 | 25% | | |
| 2 | | | | 25% | | |
| 3 | | | | 50% | | |
| 4 | | | | 100% | | |

**Final values:**
- P = ______
- I = ______
- D = ______

---

### Auto-Aim PID

| Test | P Value | I Value | D Value | Speed | Behavior | Notes |
|------|---------|---------|---------|-------|----------|-------|
| 1 | 5.0 | 0.0 | 0.0 | 25% | | |
| 2 | | | | 25% | | |
| 3 | | | | 50% | | |
| 4 | | | | 100% | | |

**Final values:**
- P = ______
- I = ______
- D = ______

---

### Snap-to-Angle PID

| Test | P Value | I Value | D Value | Speed | Behavior | Notes |
|------|---------|---------|---------|-------|----------|-------|
| 1 | 5.0 | 0.0 | 0.0 | 25% | | |
| 2 | | | | 25% | | |
| 3 | | | | 50% | | |
| 4 | | | | 100% | | |

**Final values:**
- P = ______
- I = ______
- D = ______

---

## 🎓 Understanding Each Term

### P - Proportional Gain

**What it does:** Output is proportional to error
```
Output = P × Error
```

**Example:**
- Error = 45° (robot needs to turn 45°)
- P = 5.0
- Output = 5.0 × 45° = 225° per second (clamped to max speed)

**When to increase P:**
- Robot responds too slowly
- Robot doesn't reach target
- System is "lazy"

**When to decrease P:**
- Robot oscillates
- System is "twitchy"
- Overshoots target

**Typical range:** 1.0 to 10.0 for rotation control

---

### D - Derivative Gain

**What it does:** Output is proportional to rate of change
```
Output = D × (Error - PreviousError) / dt
```

**Example:**
- Error was 45°, now 30° (approaching target)
- Rate of change = -15° per cycle
- D = 0.5
- Output = Negative (slows down rotation)

**Acts like brakes!**

**When to increase D:**
- Robot oscillates
- Need to dampen overshoot
- Want smoother motion

**When to decrease D:**
- Robot is sluggish
- Response too slow
- "Feels heavy"

**Typical range:** 0.0 to 2.0 for rotation control

---

### I - Integral Gain

**What it does:** Output accumulates over time
```
Output = I × Sum(AllPreviousErrors)
```

**When to use I:**
- Persistent steady-state error
- Robot stops 5° short and stays there
- System has friction/bias

**⚠️ For rotation control: Usually NOT needed!**

**Why avoid I:**
- Can cause "integral windup" (overshoot)
- Makes system harder to tune
- D usually solves the same problems

**If you must use I:**
- Keep very small (0.01 to 0.1)
- Add integral windup protection
- Only use after P and D are tuned

---

## 🐛 Common PID Tuning Mistakes

### Mistake 1: Tuning I First

**Wrong:**
```
Student: "It's not reaching the target. Let me increase I!"
```

**Right:**
```
Student: "It's not reaching the target. Let me increase P!"
```

**Why:** I should be the LAST thing you tune, and often you don't need it at all.

---

### Mistake 2: Not Testing at Multiple Speeds

**Wrong:**
```
Student: "Works great at 25% speed! Done!"
*Tests at 100% speed, robot goes crazy*
```

**Right:**
```
Student: "Works at 25%. Let me test at 50%, 75%, and 100% too."
```

**Why:** PID behavior changes with speed. Always test at competition speeds!

---

### Mistake 3: Changing Multiple Values at Once

**Wrong:**
```
Test 1: P=5.0, D=0.0 → Oscillates
Test 2: P=3.0, D=1.0 → Works!
Student: "Great! ...wait, which one fixed it?"
```

**Right:**
```
Test 1: P=5.0, D=0.0 → Oscillates
Test 2: P=5.0, D=0.5 → Still oscillates
Test 3: P=5.0, D=1.0 → Works!
Student: "D=1.0 fixed the oscillation!"
```

**Why:** Change one variable at a time to understand what each does.

---

### Mistake 4: Not Writing Down Results

**Wrong:**
```
Student tries 10 different values, forgets which was best.
```

**Right:**
```
Student fills out worksheet, can see progression and go back if needed.
```

**Why:** You'll forget! Document everything.

---

### Mistake 5: Tuning Without Vision Reset

**Wrong:**
```
Student tunes PID while odometry is drifted 2m from reality.
Robot behavior is unpredictable because pose is wrong.
```

**Right:**
```
Student presses BACK to force vision reset BEFORE tuning.
Odometry matches reality, robot behaves predictably.
```

**Why:** PID tuning requires accurate position feedback!

---

## 📌 Quick Reference: Default Starting Values

Copy these into Constants.java to start tuning:

```java
public static final class TeleopConstants {
    // Heading lock PID (tune first!)
    public static final double kHeadingLockP = 5.0;
    public static final double kHeadingLockI = 0.0;
    public static final double kHeadingLockD = 0.0;

    // Auto-aim PID (tune second)
    public static final double kAutoAimP = 5.0;
    public static final double kAutoAimI = 0.0;
    public static final double kAutoAimD = 0.0;

    // Snap-to-angle PID (tune third)
    public static final double kSnapToAngleP = 5.0;
    public static final double kSnapToAngleI = 0.0;
    public static final double kSnapToAngleD = 0.0;
}

public static final class AutoConstants {
    // PathPlanner PID (tune last)
    public static final PIDConstants kTranslationPID = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants kRotationPID = new PIDConstants(5.0, 0.0, 0.0);
}
```

---

## ✅ Tuning Complete Checklist

Before marking PID tuning as "done":

- [ ] Heading lock tested at 25%, 50%, 75%, 100% speeds
- [ ] Auto-aim tested at 25%, 50%, 75%, 100% speeds
- [ ] Snap-to-angle tested at 25%, 50%, 75%, 100% speeds
- [ ] Robot reaches target within 2° consistently
- [ ] No oscillation or overshoot at any speed
- [ ] Motion feels smooth and responsive
- [ ] Values documented in Constants.java
- [ ] Values committed to git with descriptive message
- [ ] All team members aware of final values
- [ ] Worksheet filled out and saved

---

## 📚 Additional Resources

**Videos:**
- [FIRST PID Tuning](https://www.youtube.com/results?search_query=frc+pid+tuning) - Search YouTube
- [Understanding PID Control](https://www.youtube.com/watch?v=wkfEZmsQqiA) - Excellent visual explanation

**Articles:**
- [WPILib PID Control](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html)
- [Chief Delphi PID Tuning](https://www.chiefdelphi.com/search?q=pid%20tuning) - Forum discussions

**Tools:**
- [PID Simulator](https://pidtuner.com/) - Practice tuning without robot
- AdvantageScope - Graph PID performance in real-time

---

## 🎯 Final Tips

1. **Start simple:** Tune at 25% speed first
2. **One at a time:** Only change one value per test
3. **Write it down:** Use the worksheet!
4. **Test thoroughly:** All speeds, multiple times
5. **Trust your eyes:** If it looks good, it probably is
6. **Don't over-tune:** "Good enough" is often perfect
7. **Save your work:** Commit to git after each successful tune
8. **Share knowledge:** Teach another student what you learned

**Remember:** PID tuning is part science, part art. Don't be afraid to experiment!

---


---

**📚 [Back to Documentation Index](README.md)**

**Last Updated:** 2026-01-16
**Game:** 2025 Reefscape
**Team:** [Your Team Number]
