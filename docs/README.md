# Documentation Index

**Welcome!** This folder contains all the guides you need to get your robot competition-ready.

---

## 🚀 Start Here

### 👥 New Student? Start with Onboarding!

**Read this first:** [**Student Onboarding Guide**](ONBOARDING_GUIDE.md)

Separate paths for freshmen vs veterans:
- 🆕 **Freshmen:** Learn FRC programming step-by-step
- 🎓 **Veterans:** Fast-track orientation for experienced students

### 🔧 Ready to Work on the Robot?

**Read this first:** [**Student Setup Checklist**](STUDENT_SETUP_CHECKLIST.md) - NEW!

Complete this checklist BEFORE starting testing:
- ✅ Required software installation
- ✅ Hardware configuration verification
- ✅ Vision system setup
- ✅ Basic functionality tests

**Read this next:** [**Start of Season Guide**](START_OF_SEASON_GUIDE.md)

Complete roadmap from hardware assembly to competition-ready robot:
- ✅ What order to do things (and why!)
- ✅ Week-by-week timeline (2-4 weeks)
- ✅ Common mistakes and how to avoid them
- ✅ When to use each guide below

---

## 📚 All Guides

### 👥 Student Resources

**[Student Onboarding Guide](ONBOARDING_GUIDE.md)** - NEW!
- For freshmen: Learn FRC programming
- For veterans: Fast-track orientation
- Learning paths, common questions, mentoring tips

---

## 🔧 Technical Guides

Follow these guides **in order** for best results:

### 1️⃣ [PID Tuning Guide](PID_TUNING_GUIDE.md)
**Do this FIRST!** (Week 1-2)

Tune rotation PID controllers for smooth motion.
- ⏱️ 3-5 days of work
- 🎯 No vision hardware required
- ✅ Must complete before vision testing

**When to use:** After basic drive works, before vision testing

---

### 2️⃣ [Vision Testing Guide](VISION_TESTING_GUIDE.md)
**Do this SECOND!** (Week 2-3)

Test vision-based autonomous commands safely.
- ⏱️ 2-3 days of work
- 🎯 Requires PID tuning to be complete
- ✅ Must complete before autonomous paths

**When to use:** After PID tuning is complete

---

### 3️⃣ [Autonomous Testing Guide](AUTO_TESTING_GUIDE.md)
**Do this THIRD!** (Week 3)

Test PathPlanner autonomous routines safely.
- ⏱️ 1-2 days of work
- 🎯 Requires PID and vision to be complete
- ✅ Two test autos included

**When to use:** After vision testing is complete

---

## ⚠️ Critical Order

```
Week 1-2: PID Tuning (FIRST!)
          ↓
Week 2-3: Vision Testing (SECOND!)
          ↓
Week 3-4: Autonomous Paths + Game Features
```

**Why?** Vision commands use PID controllers. If PID isn't tuned, vision tests will fail even if vision works perfectly!

---

## 🆘 Quick Help

**"I'm a new student, where do I start?"**
→ Read [Student Onboarding Guide](ONBOARDING_GUIDE.md)

**"I'm ready to work on the robot, where do I start?"**
→ Read [Start of Season Guide](START_OF_SEASON_GUIDE.md)

**"Robot drives but rotates jerkily"**
→ Do [PID Tuning Guide](PID_TUNING_GUIDE.md)

**"We want to test vision"**
→ First complete [PID Tuning Guide](PID_TUNING_GUIDE.md), then [Vision Testing Guide](VISION_TESTING_GUIDE.md)

**"Vision commands oscillate and fail"**
→ Go back to [PID Tuning Guide](PID_TUNING_GUIDE.md) (you skipped it!)

**"Competition is next week!"**
→ See pre-competition checklist in [Start of Season Guide](START_OF_SEASON_GUIDE.md)

---

## 📝 Guide Summary

| Guide | Purpose | Time | Prerequisites |
|-------|---------|------|---------------|
| [Student Setup Checklist](STUDENT_SETUP_CHECKLIST.md) | Verify robot configuration | 1-2 hours | None |
| [Student Onboarding](ONBOARDING_GUIDE.md) | Learn the codebase | 1-4 weeks | None |
| [Start of Season](START_OF_SEASON_GUIDE.md) | Master roadmap | Read once | None |
| [PID Tuning](PID_TUNING_GUIDE.md) | Tune controllers | 3-5 days | Setup checklist complete |
| [Vision Testing](VISION_TESTING_GUIDE.md) | Test vision commands | 2-3 days | PID tuning complete |
| [Auto Testing](AUTO_TESTING_GUIDE.md) | Test autonomous paths | 1-2 days | PID + Vision complete |

---

## 🎓 For Mentors

**Teaching tip:** Enforce the order! Don't let students skip ahead to vision testing before PID is tuned. Each guide has built-in safety checks to prevent this.

**Best practice:** Have students fill out the worksheets in each guide. Documentation is critical!

**Common issue:** Students try to test vision first → fail → frustration. Use the [Start of Season Guide](START_OF_SEASON_GUIDE.md) to explain why order matters.

---

**Last Updated:** 2026-01-16
**Game:** 2025 Reefscape (update each year!)
**Team:** [Your Team Number]
