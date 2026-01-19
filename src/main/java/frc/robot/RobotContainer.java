// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NoVisionProvider;
import frc.robot.subsystems.VisionProvider;
import frc.robot.subsystems.VisionSubsystem;

import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * ═══════════════════════════════════════════════════════════════════════════
 *                           ROBOT CONTAINER
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Purpose: Configures robot subsystems, controllers, and button mappings.
 *
 * ⚠️ WHAT TO UPDATE EACH YEAR:
 *   - Game-specific subsystems (shooter, intake, climber, etc.)
 *   - Button mappings for game-specific commands
 *   - Auto routines in createAuto()
 *   - Field layout in loadFieldLayout()
 *
 * 🧪 VISION TESTING: See VISION_TESTING_GUIDE.md for complete testing procedures.
 *
 * Controller Ports:
 *   - 0: Driver left joystick (Thrustmaster)
 *   - 1: Driver right joystick (Thrustmaster)
 *   - 2: Co-driver controller (PlayStation)
 *
 * Related files:
 *   - DriveSubsystem: Year-to-year reusable drive
 *   - VisionSubsystem: Year-to-year reusable vision
 *   - DriveCommands: Command factories for driving
 */
public class RobotContainer {

  // =========================================================================
  // SUBSYSTEMS (Year-to-year reusable)
  // =========================================================================

  /** Vision provider - Automatically falls back to NoVisionProvider if cameras unavailable */
  private final VisionProvider m_vision = createVisionProvider();

  /** Drive subsystem - Controls swerve drivetrain and tracks robot position */
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_vision);

  // =========================================================================
  // CONTROLLERS
  // =========================================================================

  /** Driver left joystick (Thrustmaster) - Translation control (forward/back, left/right) */
  private final Joystick driverLeftStick = new Joystick(HardwareConstants.kDriverLeftJoystickPort);

  /** Driver right joystick (Thrustmaster) - Rotation control */
  private final Joystick driverRightStick = new Joystick(HardwareConstants.kDriverRightJoystickPort);

  /** Co-driver PlayStation controller - Vision testing and utility commands */
  private final CommandPS5Controller coDriver = new CommandPS5Controller(HardwareConstants.kCoDriverControllerPort);

  // =========================================================================
  // TEST TARGET CONFIGURATION
  // =========================================================================

  /**
   * Test target position for vision command testing.
   *
   * Update via dashboard (Test/TargetX, Test/TargetY, Test/TargetHeading), then press START button.
   * See VISION_TESTING_GUIDE.md for setup instructions.
   */
  private Pose2d testTarget = new Pose2d(
    DriveConstants.kDefaultTestTargetX,
    DriveConstants.kDefaultTestTargetY,
    Rotation2d.fromDegrees(DriveConstants.kDefaultTestTargetHeadingDegrees)
  );

  // =========================================================================
  // AUTONOMOUS SELECTOR
  // =========================================================================

  /** Auto routine selector - Displayed on dashboard */
  private SendableChooser<Command> auto = new SendableChooser<>();

  // =========================================================================
  // CONSTRUCTOR
  // =========================================================================

  /**
   * Creates robot container and configures subsystems, controllers, and bindings.
   */
  public RobotContainer() {
    // Start camera server for dashboard video feed
    CameraServer.startAutomaticCapture();

    // ═════════════════════════════════════════════════════════════════════
    // REGISTER PATHPLANNER NAMED COMMANDS
    // ═════════════════════════════════════════════════════════════════════
    // Must be done BEFORE creating autonomous selector
    registerNamedCommands();

    // ═════════════════════════════════════════════════════════════════════
    // SET DEFAULT DRIVE COMMAND
    // ═════════════════════════════════════════════════════════════════════
    // This runs continuously during teleop when no other command is using the drive subsystem
    // Left stick = translation (X/Y), Right stick X-axis = rotation
    m_robotDrive.setDefaultCommand(
      DriveCommands.joystickDrive(
        m_robotDrive,
        () -> -driverLeftStick.getY(),   // Forward/backward (negated for intuitive control)
        () -> -driverLeftStick.getX(),   // Left/right (negated for intuitive control)
        () -> -driverRightStick.getX()   // Rotation (negated for intuitive control)
      )
    );

    // Configure button bindings
    configureButtonBindings();

    // Create autonomous selector
    createAuto();

    // Configure Shuffleboard dashboard
    configureShuffleboard();

    // ═════════════════════════════════════════════════════════════════════
    // PUBLISH TEST TARGET TO DASHBOARD
    // ═════════════════════════════════════════════════════════════════════
    // Update these values on dashboard before testing!
    SmartDashboard.putNumber("Test/TargetX", testTarget.getX());
    SmartDashboard.putNumber("Test/TargetY", testTarget.getY());
    SmartDashboard.putNumber("Test/TargetHeading", testTarget.getRotation().getDegrees());
    SmartDashboard.putNumber("Test/DistanceFromTag", DriveConstants.kDefaultDistanceFromTag);

    // ═════════════════════════════════════════════════════════════════════
    // INITIALIZE VISION MODE - Defaults to Competition Mode (safest)
    // ═════════════════════════════════════════════════════════════════════
    // Set to false = Competition Mode (strict 1m threshold)
    // Set to true = Classroom Mode (relaxed 5m threshold)
    SmartDashboard.setDefaultBoolean("Vision/ClassroomMode", false);
  }

  // =========================================================================
  // BUTTON BINDINGS
  // =========================================================================

  /**
   * Configures button->command mappings.
   *
   * 🧪 BEGINNER-FRIENDLY LAYOUT: Optimized for safe testing by inexperienced students
   *
   * DRIVER CONTROLS (Thrustmaster Joysticks):
   *   - Left stick: Translation (forward/back, left/right)
   *   - Right stick X: Rotation
   *   - Right stick Button 1: Toggle speed (full/half)
   *   - Right stick Button 2: Zero heading
   *   - Right stick Button 3: 🚨 Emergency override
   *   - Left stick Button 1: Toggle field-relative
   *
   * CO-DRIVER CONTROLS (PlayStation):
   *   FACE BUTTONS (Vision Testing):
   *     △ Triangle: Heading lock test
   *     ○ Circle: Auto-aim test
   *     ✕ Cross: Drive to distance test
   *     □ Square: X-stance (safety lock)
   *
   *   BUMPERS (Speed Control - Toggle):
   *     L1: Switch to SLOW mode (safe for practice)
   *     R1: Switch to FAST mode (when confident)
   *
   *   TRIGGERS (Rotation Snap - Hold):
   *     L2: Snap to cardinal angles (0°/90°/180°/270°)
   *     R2: Snap to diamond angles (45°/135°/225°/315°)
   *
   *   UTILITIES:
   *     Touchpad: Force vision reset
   *     Options: Reload test target from dashboard
   *     D-Pad: Reserved for future use
   *
   * ⚠️ Safety: Vision commands are autonomous (whileTrue). Driver button 3 = emergency override.
   */
  private void configureButtonBindings() {
    // ═════════════════════════════════════════════════════════════════════
    // DRIVER CONTROLS (Thrustmaster Joysticks)
    // ═════════════════════════════════════════════════════════════════════

    // Right stick button 1: Toggle speed (full/half)
    new JoystickButton(driverRightStick, 1)
        .onTrue(DriveCommands.toggleSpeed(m_robotDrive));

    // Right stick button 2: Zero heading (⚠️ face AWAY from driver station first!)
    new JoystickButton(driverRightStick, 2)
        .onTrue(DriveCommands.zeroHeading(m_robotDrive));

    // Right stick button 3: 🚨 Emergency override (cancels any running command)
    new JoystickButton(driverRightStick, 3)
        .onTrue(new Command() {
          {
            setName("EmergencyOverride");
            addRequirements(m_robotDrive);
          }

          @Override
          public void initialize() {
            System.out.println("🚨 EMERGENCY OVERRIDE - Driver has control");
          }

          @Override
          public boolean isFinished() {
            return true;
          }
        });

    // Left stick button 1: Toggle field-relative driving
    new JoystickButton(driverLeftStick, 1)
        .onTrue(DriveCommands.toggleFieldRelative(m_robotDrive));

    // ═════════════════════════════════════════════════════════════════════
    // CO-DRIVER VISION TESTING (PlayStation Controller)
    // ═════════════════════════════════════════════════════════════════════

    // Triangle button: Heading lock (driver controls translation, robot auto-rotates)
    coDriver.triangle()
        .whileTrue(Commands.deferredProxy(() ->
            DriveCommands.driveWithHeadingLock(
                m_robotDrive,
                () -> -driverLeftStick.getY(),
                () -> -driverLeftStick.getX(),
                getTestTarget(),
                0.0  // Aim with front of robot (0 degrees)
            )));

    // Circle button: Auto-aim (rotate to face test target)
    coDriver.circle()
        .whileTrue(Commands.deferredProxy(() ->
            DriveCommands.autoAimAtTarget(m_robotDrive, getTestTarget())));

    // Square button: X-stance (makes an X pattern with wheels)
    coDriver.square()
        .whileTrue(DriveCommands.xStance(m_robotDrive));

    // Cross button: Drive to distance from test target
    coDriver.cross()
        .whileTrue(Commands.deferredProxy(() ->
            DriveCommands.positionAtDistance(
                m_robotDrive,
                getTestTarget(),
                SmartDashboard.getNumber("Test/DistanceFromTag", DriveConstants.kDefaultDistanceFromTag)
            )));

    // ═════════════════════════════════════════════════════════════════════
    // CO-DRIVER SPEED CONTROL (Bumpers - Toggle mode for beginners)
    // ═════════════════════════════════════════════════════════════════════

    // L1 button: Switch to SLOW mode (safe for practice)
    coDriver.L1()
        .onTrue(DriveCommands.setSpeed(m_robotDrive, DriveConstants.kHalfSpeedMultiplier));

    // R1 button: Switch to FAST mode (when confident)
    coDriver.R1()
        .onTrue(DriveCommands.setSpeed(m_robotDrive, DriveConstants.kFullSpeedMultiplier));

    // ═════════════════════════════════════════════════════════════════════
    // CO-DRIVER ROTATION SNAP (Triggers - Hold to activate)
    // ═════════════════════════════════════════════════════════════════════

    // L2 trigger: Snap to cardinal angle (0/90/180/270°)
    coDriver.L2()
        .whileTrue(DriveCommands.snapToClosestCardinal(
            m_robotDrive,
            () -> -driverLeftStick.getY(),
            () -> -driverLeftStick.getX()
        ));

    // R2 trigger: Snap to diamond angle (45/135/225/315°)
    coDriver.R2()
        .whileTrue(DriveCommands.snapToDiamond(
            m_robotDrive,
            () -> -driverLeftStick.getY(),
            () -> -driverLeftStick.getX()
        ));

    // ═════════════════════════════════════════════════════════════════════
    // CO-DRIVER UTILITIES
    // ═════════════════════════════════════════════════════════════════════

    // Touchpad button: Force vision reset (snap odometry to AprilTag)
    coDriver.touchpad()
        .onTrue(DriveCommands.forceVisionReset(m_robotDrive));

    // Options button: Reload test target from dashboard
    coDriver.options()
        .onTrue(new Command() {
          {
            setName("UpdateTestTarget");
          }

          @Override
          public void initialize() {
            double x = SmartDashboard.getNumber("Test/TargetX", DriveConstants.kDefaultTestTargetX);
            double y = SmartDashboard.getNumber("Test/TargetY", DriveConstants.kDefaultTestTargetY);
            double heading = SmartDashboard.getNumber("Test/TargetHeading", DriveConstants.kDefaultTestTargetHeadingDegrees);
            testTarget = new Pose2d(x, y, Rotation2d.fromDegrees(heading));
            System.out.println("✅ Test target updated: (" + x + ", " + y + ", " + heading + "°)");
          }

          @Override
          public boolean isFinished() {
            return true;
          }
        }.ignoringDisable(true));

    // ═════════════════════════════════════════════════════════════════════
    // D-PAD RESERVED FOR FUTURE USE
    // ═════════════════════════════════════════════════════════════════════
    // Available for game-specific commands or preset speeds if needed
    // Example: coDriver.povUp().onTrue(intakeCommand);
  }

  /**
   * Gets current test target position.
   *
   * @return Test target pose
   */
  private Pose2d getTestTarget() {
    return testTarget;
  }

  // =========================================================================
  // AUTONOMOUS CONFIGURATION
  // =========================================================================

  /**
   * Creates autonomous selector.
   *
   * <p><b>⚠️ UPDATE EACH YEAR:</b> Add PathPlanner autos created for current game.
   *
   * <p><b>🧪 TEST AUTOS INCLUDED:</b>
   * <ul>
   *   <li><b>Test: Drive Forward:</b> Simple 2m straight line (verify basic movement)
   *   <li><b>Test: L-Shape:</b> Drive 2m, turn 90°, drive 1m (verify turning)
   * </ul>
   *
   * <p><b>How to create these paths in PathPlanner:</b>
   * <ol>
   *   <li>Install PathPlanner: https://pathplanner.dev/home.html
   *   <li>Open PathPlanner, select current season field layout
   *   <li>Create "Test Drive Forward": Start (1, 4), End (3, 4), straight line
   *   <li>Create "Test L-Shape": Start (1, 4), waypoint (3, 4), End (3, 6), 90° turn
   *   <li>Set constraints: Max Vel 2.0 m/s, Max Accel 1.5 m/s² (SAFE for testing!)
   *   <li>Save to deploy/pathplanner/autos/ folder
   * </ol>
   */
  public void createAuto() {
    auto = new SendableChooser<>();
    auto.setDefaultOption("None", null);

    // ═════════════════════════════════════════════════════════════════════
    // 🧪 TEST AUTOS - Safe, simple paths for verifying PathPlanner works
    // ═════════════════════════════════════════════════════════════════════

    try {
      // Test Auto 1: Simple straight line (2 meters forward)
      // Purpose: Verify basic path following, encoders, and gyro
      auto.addOption("🧪 Test: Drive Forward", new PathPlannerAuto("Test Drive Forward"));

      // Test Auto 2: L-shape path (forward 2m, turn 90°, forward 1m)
      // Purpose: Verify turning, rotation PID, and path transitions
      auto.addOption("🧪 Test: L-Shape", new PathPlannerAuto("Test L-Shape"));

      System.out.println("✅ Test autos loaded successfully");
    } catch (Exception e) {
      System.out.println("⚠️ Test autos not found - create them in PathPlanner first!");
      System.out.println("   See createAuto() comments for instructions");
    }

    // ═════════════════════════════════════════════════════════════════════
    // 🏆 GAME AUTOS - Add your competition autonomous routines here
    // ═════════════════════════════════════════════════════════════════════

    // TODO: Create autonomous routines for current game (update each year)
    // Examples:
    // auto.addOption("3 Piece Auto", new PathPlannerAuto("3 Piece Auto"));
    // auto.addOption("2 Piece Auto", new PathPlannerAuto("2 Piece Auto"));
    // auto.addOption("Leave Only", new PathPlannerAuto("Leave Only"));

    // Auto selector will be added to Shuffleboard in configureShuffleboard()
  }

  /**
   * Gets selected autonomous command (called by Robot.java).
   *
   * @return Selected auto command or null
   */
  public Command getAutonomousCommand() {
    return auto.getSelected();
  }

  /**
   * Gets drive subsystem instance.
   *
   * @return Drive subsystem
   */
  public DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
  }

  // =========================================================================
  // SHUFFLEBOARD CONFIGURATION
  // =========================================================================

  /**
   * Configures Shuffleboard dashboard with telemetry and command buttons.
   *
   * <p>Organized into tabs:
   * - Competition: Minimal display for drivers during matches
   * - Drive: Robot pose, speeds, field-relative mode
   * - Vision: Camera status, AprilTag detection, pose drift
   * - Tuning: Live PID tuning for practice/testing
   * - Commands: Useful buttons for testing and troubleshooting
   *
   * <p><b>📊 Telemetry Note:</b> Most telemetry uses SmartDashboard.put*() which
   * automatically appears in Shuffleboard. This method organizes the layout.
   */
  private void configureShuffleboard() {
    // ═════════════════════════════════════════════════════════════════════
    // DRIVE TAB - Main telemetry for monitoring robot state
    // ═════════════════════════════════════════════════════════════════════
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

    // Pose and heading
    driveTab.addNumber("Robot X (m)", () -> m_robotDrive.getCurrentPose().getX())
        .withPosition(0, 0).withSize(2, 1);
    driveTab.addNumber("Robot Y (m)", () -> m_robotDrive.getCurrentPose().getY())
        .withPosition(2, 0).withSize(2, 1);
    driveTab.addNumber("Heading (deg)", () -> m_robotDrive.getCurrentPose().getRotation().getDegrees())
        .withPosition(4, 0).withSize(2, 1);

    // Drive status
    driveTab.addBoolean("Field Relative", () -> m_robotDrive.getFieldRelative())
        .withPosition(0, 1).withSize(2, 1);
    driveTab.addNumber("Speed %", () -> m_robotDrive.getSpeedMultiplier() * 100)
        .withPosition(2, 1).withSize(2, 1);
    driveTab.addNumber("Battery (V)", () -> edu.wpi.first.wpilibj.RobotController.getBatteryVoltage())
        .withPosition(4, 1).withSize(2, 1);

    // Gyro
    driveTab.addNumber("Gyro Rate (deg/s)", () -> m_robotDrive.getGyroRate())
        .withPosition(0, 2).withSize(3, 1);

    // NOTE: Field visualization now in AdvantageScope via logged Pose2d data
    // See DriveSubsystem.updateDashboard() - logs "Drive/Pose" and "Drive/OdometryPose"

    // ═════════════════════════════════════════════════════════════════════
    // VISION TAB - Camera and AprilTag detection status
    // ═════════════════════════════════════════════════════════════════════
    ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");

    // Vision system global status
    visionTab.addBoolean("Vision Enabled", () -> SmartDashboard.getBoolean("Vision/Enabled", true))
        .withPosition(0, 0).withSize(2, 1);

    visionTab.addString("Overall Status", () -> {
      boolean enabled = SmartDashboard.getBoolean("Vision/Enabled", true);
      if (!enabled) return "❌ DISABLED";

      // Check both cameras
      String frontKey = "Vision/" + VisionConstants.kFrontCameraName;
      String backKey = "Vision/" + VisionConstants.kBackCameraName;

      boolean frontConnected = SmartDashboard.getBoolean(frontKey + "/Connected", false);
      boolean backConnected = SmartDashboard.getBoolean(backKey + "/Connected", false);
      int frontTargets = (int) SmartDashboard.getNumber(frontKey + "/TargetCount", 0);
      int backTargets = (int) SmartDashboard.getNumber(backKey + "/TargetCount", 0);
      int totalTargets = frontTargets + backTargets;

      if (!frontConnected && !backConnected) return "⚠️ ALL CAMERAS OFFLINE";
      if (!frontConnected || !backConnected) return "⚠️ ONE CAMERA OFFLINE";
      if (totalTargets > 0) return "✅ TRACKING " + totalTargets + " TAG(S)";
      return "🔍 NO TARGETS";
    }).withPosition(2, 0).withSize(3, 1);

    // Classroom Mode toggle - Switch between strict (competition) and relaxed (classroom) vision filtering
    visionTab.addBoolean("Classroom Mode", () -> SmartDashboard.getBoolean("Vision/ClassroomMode", false))
        .withPosition(5, 0).withSize(2, 1);

    // Mode indicator - Shows current filtering mode and thresholds
    visionTab.addString("Filter Mode", () -> {
      boolean classroomMode = SmartDashboard.getBoolean("Vision/ClassroomMode", false);
      if (classroomMode) {
        return "🏫 CLASSROOM (5m)";
      } else {
        return "🏆 COMPETITION (1m)";
      }
    }).withPosition(7, 0).withSize(3, 1);

    // ─────────────────────────────────────────────────────────────────────
    // FRONT CAMERA SECTION (Left side)
    // ─────────────────────────────────────────────────────────────────────
    String frontKey = "Vision/" + VisionConstants.kFrontCameraName;

    // Front camera header
    visionTab.addString("FRONT CAMERA", () -> "━━━━━━━━━━━━")
        .withPosition(0, 1).withSize(5, 1);

    // Front camera connection
    visionTab.addBoolean("Front Connected", () ->
        SmartDashboard.getBoolean(frontKey + "/Connected", false))
        .withPosition(0, 2).withSize(2, 1);

    // Front camera status
    visionTab.addString("Front Status", () -> {
      boolean connected = SmartDashboard.getBoolean(frontKey + "/Connected", false);
      int targets = (int) SmartDashboard.getNumber(frontKey + "/TargetCount", 0);
      if (!connected) return "⚠️ OFFLINE";
      if (targets > 0) return "✅ " + targets + " TAG(S)";
      return "🔍 NO TARGETS";
    }).withPosition(2, 2).withSize(3, 1);

    // Front detected tags
    visionTab.addString("Front Tags", () ->
        SmartDashboard.getString(frontKey + "/TagIDs", "None"))
        .withPosition(0, 3).withSize(5, 1);

    // Front average distance
    visionTab.addNumber("Front Avg Dist (m)", () ->
        SmartDashboard.getNumber(frontKey + "/AvgTagDistance", 0))
        .withPosition(0, 4).withSize(2, 1);

    // Front latency
    visionTab.addNumber("Front Latency (ms)", () ->
        SmartDashboard.getNumber(frontKey + "/LatencyMs", 0))
        .withPosition(2, 4).withSize(2, 1);

    // Front vision pose
    visionTab.addString("Front Pose", () ->
        SmartDashboard.getString(frontKey + "/EstimatedPose", "N/A"))
        .withPosition(0, 5).withSize(5, 1);

    // ─────────────────────────────────────────────────────────────────────
    // BACK CAMERA SECTION (Right side)
    // ─────────────────────────────────────────────────────────────────────
    String backKey = "Vision/" + VisionConstants.kBackCameraName;

    // Back camera header
    visionTab.addString("BACK CAMERA", () -> "━━━━━━━━━━━━")
        .withPosition(5, 1).withSize(5, 1);

    // Back camera connection
    visionTab.addBoolean("Back Connected", () ->
        SmartDashboard.getBoolean(backKey + "/Connected", false))
        .withPosition(5, 2).withSize(2, 1);

    // Back camera status
    visionTab.addString("Back Status", () -> {
      boolean connected = SmartDashboard.getBoolean(backKey + "/Connected", false);
      int targets = (int) SmartDashboard.getNumber(backKey + "/TargetCount", 0);
      if (!connected) return "⚠️ OFFLINE";
      if (targets > 0) return "✅ " + targets + " TAG(S)";
      return "🔍 NO TARGETS";
    }).withPosition(7, 2).withSize(3, 1);

    // Back detected tags
    visionTab.addString("Back Tags", () ->
        SmartDashboard.getString(backKey + "/TagIDs", "None"))
        .withPosition(5, 3).withSize(5, 1);

    // Back average distance
    visionTab.addNumber("Back Avg Dist (m)", () ->
        SmartDashboard.getNumber(backKey + "/AvgTagDistance", 0))
        .withPosition(5, 4).withSize(2, 1);

    // Back latency
    visionTab.addNumber("Back Latency (ms)", () ->
        SmartDashboard.getNumber(backKey + "/LatencyMs", 0))
        .withPosition(7, 4).withSize(2, 1);

    // Back vision pose
    visionTab.addString("Back Pose", () ->
        SmartDashboard.getString(backKey + "/EstimatedPose", "N/A"))
        .withPosition(5, 5).withSize(5, 1);

    // ═════════════════════════════════════════════════════════════════════
    // COMMANDS TAB - Useful buttons for testing/troubleshooting
    // ═════════════════════════════════════════════════════════════════════
    ShuffleboardTab commandsTab = Shuffleboard.getTab("Commands");

    // Vision commands
    commandsTab.add("🔄 Force Vision Reset", DriveCommands.forceVisionReset(m_robotDrive))
        .withPosition(0, 0).withSize(2, 1);

    // Heading commands
    commandsTab.add("🧭 Zero Heading", DriveCommands.zeroHeading(m_robotDrive))
        .withPosition(0, 1).withSize(2, 1);

    // Field-relative toggle
    commandsTab.add("🌍 Toggle Field-Relative", DriveCommands.toggleFieldRelative(m_robotDrive))
        .withPosition(0, 2).withSize(2, 1);

    // Speed presets
    commandsTab.add("🐇 Full Speed (100%)", DriveCommands.fullSpeed(m_robotDrive))
        .withPosition(2, 0).withSize(2, 1);
    commandsTab.add("🏃 Half Speed (50%)", DriveCommands.halfSpeed(m_robotDrive))
        .withPosition(2, 1).withSize(2, 1);
    commandsTab.add("🐢 Quarter Speed (25%)", DriveCommands.quarterSpeed(m_robotDrive))
        .withPosition(2, 2).withSize(2, 1);

    // Utility commands - Wheel positioning
    commandsTab.add("❌ X-Stance", DriveCommands.xStance(m_robotDrive))
        .withPosition(4, 0).withSize(2, 1);
    commandsTab.add("↑ Straight Ahead", DriveCommands.setStraightAhead(m_robotDrive))
        .withPosition(6, 0).withSize(2, 1);

    // Utility commands - Motor modes (for pit crew)
    commandsTab.add("⚙️ Coast Mode", DriveCommands.setCoastMode(m_robotDrive))
        .withPosition(4, 1).withSize(2, 1);
    commandsTab.add("🔒 Brake Mode", DriveCommands.setBrakeMode(m_robotDrive))
        .withPosition(6, 1).withSize(2, 1);

    // ═════════════════════════════════════════════════════════════════════
    // TUNING TAB - Live PID tuning for practice and testing
    // ═════════════════════════════════════════════════════════════════════
    ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");

    // Heading Lock PID (for tuning during practice)
    // These values can be modified live and are read by driveWithHeadingLockTunable()
    tuningTab.addNumber("Heading Lock kP", () -> SmartDashboard.getNumber("HeadingLock/kP", DriveConstants.kHeadingLockP))
        .withPosition(0, 0).withSize(2, 1);
    tuningTab.addNumber("Heading Lock kI", () -> SmartDashboard.getNumber("HeadingLock/kI", DriveConstants.kHeadingLockI))
        .withPosition(0, 1).withSize(2, 1);
    tuningTab.addNumber("Heading Lock kD", () -> SmartDashboard.getNumber("HeadingLock/kD", DriveConstants.kHeadingLockD))
        .withPosition(0, 2).withSize(2, 1);
    tuningTab.addNumber("Heading Lock Error (deg)", () -> SmartDashboard.getNumber("HeadingLock/ErrorDeg", 0))
        .withPosition(0, 3).withSize(2, 1);
    tuningTab.addNumber("Heading Lock Rot Speed", () -> SmartDashboard.getNumber("HeadingLock/RotSpeed", 0))
        .withPosition(0, 4).withSize(2, 1);

    // Drive telemetry for tuning
    tuningTab.addNumber("Pose Drift (m)", () -> SmartDashboard.getNumber("Drive/PoseDrift", 0))
        .withPosition(2, 0).withSize(2, 1);
    tuningTab.addString("Vision Status", () -> SmartDashboard.getString("Vision/Status", "No Data"))
        .withPosition(2, 1).withSize(2, 2);

    // Module velocities for debugging
    tuningTab.addNumber("FL Velocity (m/s)", () -> SmartDashboard.getNumber("Drive/FL_Velocity", 0))
        .withPosition(4, 0).withSize(2, 1);
    tuningTab.addNumber("FR Velocity (m/s)", () -> SmartDashboard.getNumber("Drive/FR_Velocity", 0))
        .withPosition(4, 1).withSize(2, 1);
    tuningTab.addNumber("RL Velocity (m/s)", () -> SmartDashboard.getNumber("Drive/RL_Velocity", 0))
        .withPosition(4, 2).withSize(2, 1);
    tuningTab.addNumber("RR Velocity (m/s)", () -> SmartDashboard.getNumber("Drive/RR_Velocity", 0))
        .withPosition(4, 3).withSize(2, 1);

    // Field widgets for visual feedback during tuning
    tuningTab.addString("Field: Fused", () -> "See Drive tab")
        .withPosition(6, 0).withSize(2, 1);
    tuningTab.addString("Field: Pure Odometry", () -> "See Drive tab")
        .withPosition(6, 1).withSize(2, 1);

    // ═════════════════════════════════════════════════════════════════════
    // COMPETITION TAB - Minimal display for drivers during matches
    // ═════════════════════════════════════════════════════════════════════
    ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

    // Most critical info only
    compTab.addNumber("Battery (V)", () -> edu.wpi.first.wpilibj.RobotController.getBatteryVoltage())
        .withPosition(0, 0).withSize(2, 2);

    compTab.addBoolean("Field Relative", () -> m_robotDrive.getFieldRelative())
        .withPosition(2, 0).withSize(2, 1);

    compTab.addNumber("Speed %", () -> m_robotDrive.getSpeedMultiplier() * 100)
        .withPosition(2, 1).withSize(2, 1);

    compTab.addString("Vision", () -> {
      boolean enabled = SmartDashboard.getBoolean("Vision/Enabled", true);
      if (!enabled) return "OFF";

      // Check both cameras
      String frontKey = "Vision/" + VisionConstants.kFrontCameraName;
      String backKey = "Vision/" + VisionConstants.kBackCameraName;

      boolean frontConnected = SmartDashboard.getBoolean(frontKey + "/Connected", false);
      boolean backConnected = SmartDashboard.getBoolean(backKey + "/Connected", false);
      int frontTargets = (int) SmartDashboard.getNumber(frontKey + "/TargetCount", 0);
      int backTargets = (int) SmartDashboard.getNumber(backKey + "/TargetCount", 0);
      int totalTargets = frontTargets + backTargets;

      // Show status based on both cameras
      if (!frontConnected && !backConnected) return "ALL OFF";
      if (!frontConnected || !backConnected) return "1 OFF";
      if (totalTargets > 0) return "OK (" + totalTargets + ")";
      return "NO TAGS";
    }).withPosition(4, 0).withSize(1, 2);

    // Auto selector
    compTab.add("Auto Mode", auto)
        .withPosition(5, 0).withSize(2, 2);

    // NOTE: Field visualization now in AdvantageScope via logged Pose2d data
  }

  // =========================================================================
  // VISION INITIALIZATION
  // =========================================================================

  /**
   * Creates vision provider with automatic fallback if vision unavailable.
   *
   * @return VisionSubsystem or NoVisionProvider (fallback)
   */
  private static VisionProvider createVisionProvider() {
    try {
      // Step 1: Load field layout for current game
      AprilTagFieldLayout fieldLayout = loadFieldLayout();

      if (fieldLayout == null) {
        DriverStation.reportWarning(
          "Field layout not loaded - using NoVisionProvider",
          false
        );
        return new NoVisionProvider();
      }

      // Step 2: Create camera configurations for front and back cameras
      Transform3d frontCameraTransform = createFrontCameraTransform();
      Transform3d backCameraTransform = createBackCameraTransform();

      // Step 3: Create VisionSubsystem with BOTH cameras
      return new VisionSubsystem(
        fieldLayout,
        new VisionSubsystem.CameraConfig(VisionConstants.kFrontCameraName, frontCameraTransform),
        new VisionSubsystem.CameraConfig(VisionConstants.kBackCameraName, backCameraTransform)
      );

    } catch (Exception e) {
      DriverStation.reportError(
        "Failed to initialize vision: " + e.getMessage(),
        false
      );
      return new NoVisionProvider();
    }
  }

  /**
   * Loads AprilTag field layout for current game.
   *
   * <p><b>⚠️ UPDATE EACH YEAR:</b> Replace with current season's field layout.
   * <p>Available fields are defined in {@code AprilTagFields} enum.
   * <p>Examples: k2024Crescendo, k2023ChargedUp, k2025Reefscape, etc.
   *
   * @return Field layout or null if unavailable
   */
  private static AprilTagFieldLayout loadFieldLayout() {
    try {
      // TODO: Update to current season field layout
      // Example: return AprilTagFieldLayout.loadField(AprilTagFields.k[YEAR][GameName]);
      throw new UnsupportedOperationException(
        "Field layout not configured! Update RobotContainer.loadFieldLayout() with current season field."
      );
    } catch (Exception e) {
      DriverStation.reportWarning(
        "Could not load AprilTag field layout: " + e.getMessage() +
        " - Vision will be disabled",
        false
      );
      return null;
    }
  }

  /**
   * Creates front camera-to-robot transform from VisionConstants.
   *
   * <p><b>⚠️ UPDATE VisionConstants EACH YEAR</b> with measured camera position/orientation.
   *
   * @return Transform3d from robot center to front camera
   */
  private static Transform3d createFrontCameraTransform() {
    return new Transform3d(
      new Translation3d(
        VisionConstants.kFrontCameraToRobotX,
        VisionConstants.kFrontCameraToRobotY,
        VisionConstants.kFrontCameraToRobotZ
      ),
      new Rotation3d(
        VisionConstants.kFrontCameraRollRadians,
        VisionConstants.kFrontCameraPitchRadians,
        VisionConstants.kFrontCameraYawRadians
      )
    );
  }

  /**
   * Creates back camera-to-robot transform from VisionConstants.
   *
   * <p><b>⚠️ UPDATE VisionConstants EACH YEAR</b> with measured camera position/orientation.
   *
   * @return Transform3d from robot center to back camera
   */
  private static Transform3d createBackCameraTransform() {
    return new Transform3d(
      new Translation3d(
        VisionConstants.kBackCameraToRobotX,
        VisionConstants.kBackCameraToRobotY,
        VisionConstants.kBackCameraToRobotZ
      ),
      new Rotation3d(
        VisionConstants.kBackCameraRollRadians,
        VisionConstants.kBackCameraPitchRadians,
        VisionConstants.kBackCameraYawRadians
      )
    );
  }

  /**
   * Legacy method for single camera (deprecated).
   *
   * @deprecated Use createFrontCameraTransform() or createBackCameraTransform() instead
   */
  @Deprecated
  private static Transform3d createCameraTransform() {
    return createFrontCameraTransform();
  }

  // =========================================================================
  // PATHPLANNER NAMED COMMANDS
  // =========================================================================

  /**
   * Registers named commands for use in PathPlanner autonomous routines.
   *
   * <p><b>What are named commands?</b> Custom actions that can be inserted into
   * autonomous paths (e.g., run intake, shoot, deploy mechanism).
   *
   * <p><b>How to use:</b>
   * <ol>
   *   <li>Register command here with a name
   *   <li>In PathPlanner GUI, add "Named Command" marker to path
   *   <li>Select registered command name
   *   <li>Command runs when robot reaches that point in path
   * </ol>
   *
   * <p><b>⚠️ UPDATE EACH YEAR:</b> Add game-specific commands here!
   */
  private void registerNamedCommands() {
    // Import required for PathPlanner named commands
    com.pathplanner.lib.auto.NamedCommands.registerCommand(
      "Print Start",
      Commands.runOnce(() -> System.out.println("🚀 Auto started!"))
    );

    com.pathplanner.lib.auto.NamedCommands.registerCommand(
      "Print End",
      Commands.runOnce(() -> System.out.println("✅ Auto complete!"))
    );

    // ═══════════════════════════════════════════════════════════════════════
    // 🏆 GAME-SPECIFIC NAMED COMMANDS
    // ═══════════════════════════════════════════════════════════════════════
    // TODO: Register game-specific commands for use in PathPlanner autonomous
    //
    // Examples (update for current game):
    // NamedCommands.registerCommand("Intake Game Piece", intakeSubsystem.runIntake());
    // NamedCommands.registerCommand("Score Game Piece", scorerSubsystem.score());
    // NamedCommands.registerCommand("Deploy Mechanism", deploySubsystem.deploy());
  }
}

