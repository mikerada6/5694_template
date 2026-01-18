# Student Setup Checklist

**Purpose:** Ensure your robot codebase is properly configured before starting drive and vision testing.

**When to use:** Complete this checklist BEFORE attempting PID tuning or vision testing.

---

## ✅ Pre-Season Configuration Checklist

### 1️⃣ Required Software Installation

- [ ] **Java 17 (JDK 17)** installed
  - Download from: https://adoptium.net/
  - Verify: Run `java -version` in terminal (should show "17.x.x")
  - ⚠️ **Not Java 21 or 25!** FRC requires Java 17

- [ ] **WPILib 2025** installed
  - Download from: https://github.com/wpilibsuite/allwpilib/releases
  - Includes VS Code with FRC extensions

- [ ] **Git** installed
  - Verify: Run `git --version` in terminal

- [ ] **PhotonVision** installed on coprocessor
  - Download from: https://photonvision.org/
  - Access UI at: http://photonvision.local:5800

- [ ] **PathPlanner** installed
  - Download from: https://pathplanner.dev/
  - Used to create autonomous paths

---

### 2️⃣ Hardware Configuration

#### Swerve Module CAN IDs

Open: `src/main/java/frc/robot/constants/HardwareConstants.java`

Verify these match your physical robot wiring:

- [ ] **Front Left** - Driving: `10`, Turning: `11`
- [ ] **Front Right** - Driving: `20`, Turning: `21`
- [ ] **Rear Right** - Driving: `30`, Turning: `31`
- [ ] **Rear Left** - Driving: `40`, Turning: `41`

**If different:** Update the `kFrontLeftDrivingCanId`, `kFrontLeftTurningCanId`, etc. in HardwareConstants.java

#### Swerve Module Angular Offsets

- [ ] **Calibrate absolute encoders** using REV Hardware Client
  - Each module should have bevel gear facing specific direction:
    - Front Left: Facing left (-90°)
    - Front Right: Facing front (0°)
    - Rear Left: Facing back (180°)
    - Rear Right: Facing right (90°)

- [ ] **Update offsets** in HardwareConstants.java if needed:
  ```java
  kFrontLeftChassisAngularOffset = -Math.PI / 2;
  kFrontRightChassisAngularOffset = 0;
  kBackLeftChassisAngularOffset = Math.PI;
  kBackRightChassisAngularOffset = Math.PI / 2;
  ```

#### Robot Physical Dimensions

- [ ] **Measure wheelbase and track width**
  - Wheelbase: Distance between front and rear wheels
  - Track width: Distance between left and right wheels
  - Update in HardwareConstants.java if not 29 inches:
    ```java
    kTrackWidth = Units.inchesToMeters(29);
    kWheelBase = Units.inchesToMeters(29);
    ```

- [ ] **Measure wheel diameter** (update if wheels wear down)
  - Default: 3.0 inches
  - Update `kWheelDiameterMeters` in HardwareConstants.java

#### Controller Ports

- [ ] **Verify controller USB ports** match code:
  - Left joystick → Port 0
  - Right joystick → Port 1
  - PlayStation controller → Port 2
  - Update `kDriverLeftJoystickPort`, etc. in HardwareConstants.java if different

---

### 3️⃣ Vision System Configuration

Open: `src/main/java/frc/robot/constants/VisionConstants.java`

#### Camera Name

- [ ] **Update camera name** to match PhotonVision:
  ```java
  public static final String kCameraName = "YOUR_CAMERA_NAME";
  ```
  - Find actual name at: http://photonvision.local:5800
  - Common names: "photonvision", "OV9281", "Microsoft_LifeCam"

#### Camera Position (Robot-to-Camera Transform)

- [ ] **Measure camera position** from robot center:
  - X: Forward (+) / Backward (-)
  - Y: Left (+) / Right (-)
  - Z: Up (+) / Down (-)

  Update in VisionConstants.java:
  ```java
  kCameraToRobotX = 0.3;  // Example: 0.3m forward
  kCameraToRobotY = 0.0;  // Example: centered
  kCameraToRobotZ = 0.5;  // Example: 0.5m up
  ```

#### Camera Orientation

- [ ] **Measure camera tilt angle** (pitch):
  - Negative = tilted down (common)
  - Positive = tilted up
  - Example: -30° = -0.524 radians

  Update in VisionConstants.java:
  ```java
  kCameraPitchRadians = Math.toRadians(-30);  // Example: 30° down
  ```

- [ ] **Set yaw and roll** (usually 0.0 unless camera is rotated):
  ```java
  kCameraYawRadians = 0.0;   // 0 = facing forward
  kCameraRollRadians = 0.0;  // 0 = upright
  ```

---

### 4️⃣ Field Layout Verification

Open: `src/main/java/frc/robot/RobotContainer.java` (around line 427)

- [ ] **Verify field layout** loads correctly:
  ```java
  return AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
  ```

- [ ] **Check console output** when code deploys:
  - Should NOT see: "Could not load AprilTag field layout"
  - Should see: Field layout loaded successfully

---

### 5️⃣ Build and Deploy Verification

- [ ] **Open project** in VS Code
  - File → Open Folder → Select `2025-reefscape-main`

- [ ] **Build the code** (without deploying):
  - Press `Ctrl+Shift+P` (Windows/Linux) or `Cmd+Shift+P` (Mac)
  - Type: "WPILib: Build Robot Code"
  - Should see: "Build Successful"

- [ ] **Connect to robot** (via USB or WiFi)
  - Robot should be powered on
  - Driver Station connected

- [ ] **Deploy code**:
  - Press `Ctrl+Shift+P` / `Cmd+Shift+P`
  - Type: "WPILib: Deploy Robot Code"
  - Should see: "Deploy Successful"

- [ ] **Open Driver Station** and verify:
  - Communications: Green
  - Robot Code: Green
  - Joysticks: Green (all controllers connected)

---

### 6️⃣ Dashboard Verification

Open SmartDashboard or Shuffleboard:

- [ ] **Check Drive values appear:**
  - `Drive/Heading` - Shows current gyro angle
  - `Drive/SpeedMultiplier` - Shows 1.0 (full speed)
  - `Drive/FieldRelative` - Shows true
  - `Drive/FL_Velocity`, `FR_Velocity`, etc. - Shows module speeds

- [ ] **Check Vision values appear:**
  - `Vision/Enabled` - Shows true
  - `Vision/photonvision/Connected` - Shows true (if camera working)
  - `Vision/Status` - Shows current status

- [ ] **Check Field widgets:**
  - `Field/Fused` - Shows robot position (should be at 0,0)
  - `Field/PureOdometry` - Shows odometry-only position

- [ ] **Check Autonomous selector:**
  - `Autonomous Command` - Shows dropdown with test autos:
    - "None"
    - "🧪 Test: Drive Forward"
    - "🧪 Test: L-Shape"

---

### 7️⃣ Basic Drive Test (No Vision)

- [ ] **Enable teleop mode** in Driver Station

- [ ] **Test joystick control:**
  - Left stick → Robot translates (forward/back/strafe)
  - Right stick X-axis → Robot rotates
  - All four swerve modules should respond

- [ ] **Test field-relative mode:**
  - Press left stick button 1 to toggle
  - Dashboard should show `Drive/FieldRelative` changes
  - Forward on joystick should always go "away from driver station"

- [ ] **Test speed toggle:**
  - Press right stick button 1
  - Dashboard `Drive/SpeedMultiplier` should change (1.0 ↔ 0.5)

- [ ] **Test zero heading:**
  - Point robot away from driver station
  - Press right stick button 2
  - Dashboard `Drive/Heading` should show ~0°

---

### 8️⃣ Vision System Test (Basic)

- [ ] **Check camera connection:**
  - Dashboard `Vision/photonvision/Connected` should be `true`
  - If false: Check USB cable, coprocessor power, PhotonVision running

- [ ] **Check AprilTag detection:**
  - Place robot ~2 meters from AprilTag
  - Dashboard `Vision/photonvision/TargetCount` should show 1 or more
  - Field widget `Field/VisionOnly` should show robot near correct position

- [ ] **Test vision kill switch:**
  - Dashboard: Set `Vision/Enabled` to `false`
  - Vision should stop updating (intentional)
  - Set back to `true` to re-enable

---

## ✅ Checklist Complete!

Once all items are checked, you're ready to proceed to:

**Next Step:** [PID Tuning Guide](PID_TUNING_GUIDE.md)

---

## ⚠️ Common Issues

### Build Errors

**Error: "Unsupported class file major version 69"**
- **Problem:** Java 25 installed instead of Java 17
- **Fix:** Install Java 17 from https://adoptium.net/

**Error: "Could not find package frc.robot.constants"**
- **Problem:** Old code or incorrect file structure
- **Fix:** Ensure all constants files are in `src/main/java/frc/robot/constants/`

### Vision Issues

**Camera shows "Not Connected"**
- Check USB cable from camera to coprocessor
- Verify PhotonVision is running (http://photonvision.local:5800)
- Check coprocessor power

**Vision sees tags but position is wrong**
- Camera transform (X/Y/Z position) needs calibration
- Camera pitch angle needs measurement
- Return to Vision Configuration section above

### Drive Issues

**Robot drives backwards when joystick pushed forward**
- Check gyro orientation in DriveSubsystem.java line 800
- May need to negate gyro angle

**Robot drifts even with joystick released**
- Increase joystick deadband in DriveConstants.java
- Default is 0.1, try 0.15

**Swerve modules point wrong directions**
- Chassis angular offsets need calibration
- Use REV Hardware Client to verify absolute encoder values

---

**Last Updated:** 2026-01-16
**Next:** [PID Tuning Guide](PID_TUNING_GUIDE.md)
