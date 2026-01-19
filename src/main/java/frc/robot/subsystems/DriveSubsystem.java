package frc.robot.subsystems;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.HardwareConstants;

/**
 * ═══════════════════════════════════════════════════════════════════════════
 *                           DRIVE SUBSYSTEM
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * ✅ YEAR-TO-YEAR REUSABLE - This class should NOT need changes between seasons!
 *
 * Purpose: Controls the robot's swerve drivetrain and tracks position on field.
 *
 * Features:
 *   ✅ MAXSwerve drivetrain control (4 independent modules)
 *   ✅ Dual pose tracking (pose estimator with vision + pure odometry backup)
 *   ✅ Vision fusion (AprilTag corrections improve accuracy)
 *   ✅ Field-relative and robot-relative driving modes
 *   ✅ Speed control (full/half/custom multiplier)
 *   ✅ Defensive X-stance positioning
 *   ✅ PathPlanner integration for autonomous
 *   ✅ Brownout protection
 *   ✅ Comprehensive dashboard diagnostics
 *
 * ⚠️ WHAT TO UPDATE EACH YEAR:
 *
 *   1. In Constants.DriveConstants:
 *      - CAN IDs for swerve modules (if wiring changes)
 *      - kTrackWidth and kWheelBase (measure your robot!)
 *      - Chassis angular offsets (calibrate with Phoenix Tuner)
 *      - kMaxSpeedMetersPerSecond (tune for your robot's max speed)
 *
 *   2. In Constants.VisionConstants:
 *      - Pose estimator standard deviations (how much to trust vision vs encoders)
 *
 *   3. Gyro orientation: If gyro mounted differently, check getRotation2d()
 *
 * ⚠️ DO NOT MODIFY THIS FILE unless changing core drive functionality!
 *
 * How Pose Tracking Works:
 *   1. Encoders + Gyro: Measure how far wheels have turned and robot rotation
 *   2. Odometry: Calculate where robot SHOULD be based on wheel movements
 *   3. Vision: When AprilTags detected, correct accumulated odometry errors
 *   4. Pose Estimator: Fuses all sources for most accurate position
 *   5. Pure Odometry: Backup tracking (no vision) for comparison
 *
 * Why Two Tracking Systems?
 *   - Pose Estimator (Fused): Most accurate, used for driving
 *   - Pure Odometry (Ghost): Backup, helps diagnose vision issues
 *   - If they drift apart on dashboard, vision might be miscalibrated!
 *
 * Dashboard Keys (logged to AdvantageKit + SmartDashboard):
 *   - Drive/Pose - Fused robot pose (vision + odometry) - Pose2d struct
 *   - Drive/OdometryPose - Pure odometry pose (no vision) - Pose2d struct
 *   - Drive/ModuleStates - All 4 swerve module states - SwerveModuleState[] array
 *   - Drive/FieldRelative - Field-relative (true) or robot-relative (false)
 *   - Drive/SpeedMultiplier - Current speed percentage (0.0 to 1.0)
 *   - Drive/Heading - Robot rotation in degrees
 *   - Drive/VelocityX/Y/Omega - Robot chassis velocities
 *   - Drive/BatteryVoltage - Current battery voltage
 *   - Drive/BrownoutRisk - Warning if voltage < 6.5V
 *
 * AdvantageScope Features:
 *   - 3D Field widget: Compare "Drive/Pose" vs "Drive/OdometryPose" to see drift
 *   - Module state graphs: AdvantageScope auto-graphs "Drive/ModuleStates" array
 *   - Post-match replay: Logs saved to /U/logs/ on robot (USB stick)
 *
 * Pre-Match Checklist:
 *   1. Run zeroHeading() command facing away from driver station
 *   2. Verify AdvantageScope shows robot at correct starting position (Drive/Pose)
 *   3. Check all 4 module velocities respond to joystick
 *   4. Verify Vision/[Camera]/Connected is true
 *   5. Compare Drive/Pose vs Drive/OdometryPose in AdvantageScope (drift < 0.5m)
 *
 * Related files:
 *   - VisionProvider: Interface for vision integration
 *   - MAXSwerveModule: Individual swerve module controller
 *   - DriveCommands: Command factories for driving
 */
public class DriveSubsystem extends SubsystemBase {

    // =========================================================================
    // HARDWARE COMPONENTS
    // ⚠️ CAN IDs and offsets configured in Constants.DriveConstants
    // =========================================================================

    /**
     * Swerve modules - 4 independent drive + steering units.
     *
     * <p><b>⚠️ UPDATE CAN IDs IN Constants.DriveConstants, NOT HERE!</b>
     *
     * <p><b>Module order:</b> Front-Left, Front-Right, Rear-Left, Rear-Right
     * This order MUST match DriveConstants.kDriveKinematics definition!
     */
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
        HardwareConstants.kFrontLeftDrivingCanId,
        HardwareConstants.kFrontLeftTurningCanId,
        HardwareConstants.kFrontLeftChassisAngularOffset
    );

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
        HardwareConstants.kFrontRightDrivingCanId,
        HardwareConstants.kFrontRightTurningCanId,
        HardwareConstants.kFrontRightChassisAngularOffset
    );

    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
        HardwareConstants.kRearLeftDrivingCanId,
        HardwareConstants.kRearLeftTurningCanId,
        HardwareConstants.kBackLeftChassisAngularOffset
    );

    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
        HardwareConstants.kRearRightDrivingCanId,
        HardwareConstants.kRearRightTurningCanId,
        HardwareConstants.kBackRightChassisAngularOffset
    );

    /**
     * NavX gyroscope - Measures robot rotation.
     *
     * <p><b>What it does:</b> Tracks which direction robot is facing (0-360°)
     *
     * <p><b>Why we need it:</b>
     * <ul>
     *   <li>Field-relative driving (forward always goes "away from driver")
     *   <li>Accurate pose estimation (wheel slip causes encoder drift)
     *   <li>Auto-aim commands (rotate to face targets)
     * </ul>
     *
     * <p><b>⚠️ Connection:</b> Plugged into roboRIO MXP SPI port
     */
    private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    // =========================================================================
    // POSE TRACKING SYSTEMS
    // =========================================================================

    /**
     * PRIMARY TRACKER: "The Truth" - Fuses encoders + gyro + vision.
     *
     * <p><b>What it does:</b> Estimates robot position by combining:
     * <ul>
     *   <li>Wheel encoder distances (measures how far wheels turned)
     *   <li>Gyro angle (measures which way robot rotated)
     *   <li>Vision corrections (AprilTag detections fix accumulated errors)
     * </ul>
     *
     * <p><b>Why fusion?</b>
     * <ul>
     *   <li>Encoders drift over time (wheel slip, carpet friction)
     *   <li>Vision is accurate but intermittent (tags not always visible)
     *   <li>Combining both gives best estimate
     * </ul>
     *
     * <p><b>Used for:</b> All driving commands, auto pathing, game-specific positioning
     */
    private final SwerveDrivePoseEstimator m_poseEstimator;

    /**
     * BACKUP TRACKER: "The Ghost" - Pure encoders + gyro (no vision).
     *
     * <p><b>What it does:</b> Tracks position using ONLY wheel encoders and gyro.
     * Ignores vision completely.
     *
     * <p><b>Why have this?</b>
     * <ul>
     *   <li>Compare against fused estimate to diagnose vision problems
     *   <li>If they drift apart significantly, vision might be misconfigured
     *   <li>Brownout protection (if vision crashes, we still have position estimate)
     * </ul>
     *
     * <p><b>Dashboard:</b> Check "Drive/PoseDrift" - should stay under 0.5m during match
     */
    private final SwerveDriveOdometry m_odometry;

    /**
     * Vision system - Provides AprilTag-based pose corrections.
     *
     * <p><b>Interface pattern:</b> VisionProvider allows dependency injection:
     * <ul>
     *   <li>VisionSubsystem = Real PhotonVision cameras
     *   <li>NoVisionProvider = Fallback when cameras unavailable
     * </ul>
     */
    private final VisionProvider m_vision;

    // =========================================================================
    // ROBOT STATE
    // =========================================================================

    /**
     * Speed multiplier - Scales all drive commands.
     *
     * <p><b>Values:</b>
     * <ul>
     *   <li>1.0 = Full speed (4.8 m/s default)
     *   <li>0.5 = Half speed (2.4 m/s)
     *   <li>0.0 = Stopped
     * </ul>
     *
     * <p><b>Set via:</b> DriveCommands.setSpeed(), toggleSpeed(), etc.
     */
    private double m_speedMultiplier = DriveConstants.kDefaultSpeedMultiplier;

    /**
     * Field-relative driving mode (true = field-relative, false = robot-relative).
     *
     * <p><b>Field-relative (true):</b> Forward is always "away from driver station"
     * <p><b>Robot-relative (false):</b> Forward is robot's front
     *
     * <p><b>Set via:</b> DriveCommands.toggleFieldRelative()
     */
    private boolean m_fieldRelative = true;

    // =========================================================================
    // ROBUSTNESS MONITORING
    // =========================================================================

    /** Tracks if battery voltage is critically low (< 6.5V) */
    private boolean m_isBrownedOut = false;

    /** Timestamp of last successful vision update (for timeout detection) */
    private double m_lastVisionUpdateTime = 0.0;

    /** Voltage threshold for brownout warnings (Volts) - From Constants.DriveConstants */
    private static final double kBrownoutVoltageThreshold = DriveConstants.kBrownoutVoltageThreshold;

    /** How long to wait before warning about no vision updates (seconds) - From Constants.DriveConstants */
    private static final double kVisionTimeoutSeconds = DriveConstants.kVisionTimeoutSeconds;

    // =========================================================================
    // CONSTRUCTOR
    // ⚠️ DO NOT CALL DIRECTLY - Called from RobotContainer
    // =========================================================================

    /**
     * Creates drive subsystem with vision-based pose estimation.
     *
     * <p><b>Initialization order:</b>
     * <ol>
     *   <li>Store vision provider reference
     *   <li>Initialize pose estimator with starting values
     *   <li>Initialize pure odometry backup tracker
     *   <li>Configure PathPlanner for autonomous
     *   <li>Add dashboard widgets
     * </ol>
     *
     * <p><b>⚠️ Called from:</b> RobotContainer.java
     *
     * @param vision VisionProvider for AprilTag corrections (VisionSubsystem or NoVisionProvider)
     */
    public DriveSubsystem(VisionProvider vision) {
        this.m_vision = vision;

        // NOTE: We removed Thread.sleep and gyro.reset() from constructor
        // Reason: Gyro calibrates on boot automatically. Sleeping delays robot startup.
        // If gyro needs zeroing, use zeroHeading() command after robot is enabled.

        // ─────────────────────────────────────────────────────────────────────
        // Initialize PRIMARY tracker (Vision + Encoders + Gyro)
        // ─────────────────────────────────────────────────────────────────────
        m_poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,  // Swerve kinematics (wheelbase/trackwidth)
            getRotation2d(),                   // Current gyro angle
            getModulePositions(),              // Current wheel encoder positions
            new Pose2d(),                      // Starting pose (0,0,0°) - reset in auto/teleop

            // Standard deviations for encoder/gyro measurements [x, y, theta]
            // Lower = trust more. These say "trust encoders/gyro quite a bit"
            VecBuilder.fill(
                DriveConstants.kStateStdDevX,
                DriveConstants.kStateStdDevY,
                DriveConstants.kStateStdDevTheta
            ),

            // Standard deviations for vision measurements [x, y, theta]
            // These are placeholders - overwritten per frame in processVisionMeasurements()
            VecBuilder.fill(
                DriveConstants.kVisionStdDevX,
                DriveConstants.kVisionStdDevY,
                DriveConstants.kVisionStdDevTheta
            )
        );

        // ─────────────────────────────────────────────────────────────────────
        // Initialize BACKUP tracker (Encoders + Gyro only, no vision)
        // ─────────────────────────────────────────────────────────────────────
        m_odometry = new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            getRotation2d(),
            getModulePositions()
        );

        // Configure PathPlanner for autonomous path following
        configurePathPlanner();
    }

    // =========================================================================
    // PERIODIC - MAIN UPDATE LOOP (Called every 20ms)
    // =========================================================================

    /**
     * Main update loop - runs every 20ms (50 times per second).
     *
     * <p><b>Execution order is critical!</b>
     * <ol>
     *   <li>Check battery voltage (brownout protection)
     *   <li>Read current sensor values (gyro angle, wheel positions)
     *   <li>Update both trackers with physics (encoder/gyro measurements)
     *   <li>Process vision measurements (if available)
     *   <li>Update dashboard diagnostics
     * </ol>
     *
     * <p><b>Why this order?</b>
     * <ul>
     *   <li>Brownout check first (critical safety)
     *   <li>Sensor reads before updates (need fresh data)
     *   <li>Physics before vision (vision corrections applied on top)
     *   <li>Dashboard last (all data ready to display)
     * </ul>
     */
    @Override
    public void periodic() {
        // Step 0: Check for low battery voltage
        checkBrownout();

        // Step 1: Get fresh sensor readings
        Rotation2d gyroAngle = getRotation2d();
        SwerveModulePosition[] modulePositions = getModulePositions();

        // Step 2: Update both trackers with physics (wheel encoders + gyro)
        m_poseEstimator.update(gyroAngle, modulePositions);
        m_odometry.update(gyroAngle, modulePositions);

        // Step 3: Add vision corrections (if AprilTags visible and safe to use)
        processVisionMeasurements();

        // Step 4: Update dashboard with all diagnostic info
        updateDashboard();
    }

    // =========================================================================
    // ROBUSTNESS: BROWNOUT DETECTION
    // =========================================================================

    /**
     * Monitors battery voltage and warns if brownout is imminent.
     *
     * <p><b>What is a brownout?</b> When battery voltage drops below ~6.3V, the
     * roboRIO automatically shuts down to protect itself. This is BAD during a match!
     *
     * <p><b>Common causes:</b>
     * <ul>
     *   <li>Too many motors running at once
     *   <li>Old/weak battery
     *   <li>Poor battery connections
     *   <li>Driving + shooting + climbing simultaneously
     * </ul>
     *
     * <p><b>What this does:</b>
     * <ul>
     *   <li>Logs warning to DriverStation when voltage < 6.5V
     *   <li>Updates dashboard so pit crew can see battery health
     *   <li>Prevents repeated warnings (only warns once per brownout event)
     * </ul>
     *
     * <p><b>Dashboard:</b> Watch "Drive/BatteryVoltage" - should stay above 10V during match
     */
    private void checkBrownout() {
        double voltage = edu.wpi.first.wpilibj.RobotController.getBatteryVoltage();

        // Entering brownout danger zone
        if (voltage < kBrownoutVoltageThreshold) {
            // Only warn once when we first enter brownout (not every loop)
            if (!m_isBrownedOut) {
                DriverStation.reportWarning(
                    "Low voltage detected: " + String.format("%.1f", voltage) + "V - Brownout risk!",
                    false  // Don't print stack trace
                );
                m_isBrownedOut = true;
            }
        } else {
            // Voltage recovered - reset flag so we warn again if it drops
            m_isBrownedOut = false;
        }

        // NOTE: Battery voltage and brownout status are logged in updateDashboard()
    }

    // =========================================================================
    // DASHBOARD DIAGNOSTICS
    // =========================================================================

    /**
     * Updates AdvantageKit logs and Shuffleboard with robot state and diagnostics.
     *
     * <p><b>AdvantageKit Structured Logging:</b>
     * <ul>
     *   <li><b>Pose2d structs:</b> Fused pose (vision-corrected) + Pure odometry pose
     *       <ul><li>AdvantageScope automatically displays in 3D field widget</li>
     *           <li>AdvantageScope can calculate pose drift automatically</li></ul>
     *   <li><b>SwerveModuleState[] array:</b> All 4 module states (angle + velocity)
     *       <ul><li>AdvantageScope automatically graphs velocities</li>
     *           <li>Easier to spot broken modules than individual values</li></ul>
     *   <li><b>Robot velocities:</b> Chassis speeds (vx, vy, omega)
     *   <li><b>Battery voltage:</b> Monitor brownout risk
     * </ul>
     *
     * <p><b>Shuffleboard Compatibility:</b>
     * <ul>
     *   <li>Critical values still sent to SmartDashboard for Shuffleboard widgets
     *   <li>Field-relative mode, speed multiplier, etc. needed for driver display
     * </ul>
     *
     * <p><b>Key diagnostic: Compare "Drive/Pose" vs "Drive/OdometryPose" in AdvantageScope</b>
     * <ul>
     *   <li>< 0.3m drift = Good (vision and odometry agree)
     *   <li>0.3-0.5m drift = Okay (some drift, acceptable)
     *   <li>> 0.5m drift = Problem! (vision misconfigured or bad detections)
     * </ul>
     */
    private void updateDashboard() {
        // ═════════════════════════════════════════════════════════════════════
        // ADVANTAGEKIT STRUCTURED LOGGING
        // ═════════════════════════════════════════════════════════════════════

        // Log poses as Pose2d structs (AdvantageScope will show in 3D field widget)
        Logger.recordOutput("Drive/Pose", getCurrentPose());
        Logger.recordOutput("Drive/OdometryPose", m_odometry.getPoseMeters());

        // Log all module states as array (AdvantageScope will graph velocities)
        SwerveModuleState[] moduleStates = new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
        };
        Logger.recordOutput("Drive/ModuleStates", moduleStates);

        // Log robot velocities
        ChassisSpeeds speeds = getRobotRelativeSpeeds();
        Logger.recordOutput("Drive/VelocityX", speeds.vxMetersPerSecond);
        Logger.recordOutput("Drive/VelocityY", speeds.vyMetersPerSecond);
        Logger.recordOutput("Drive/VelocityOmega", speeds.omegaRadiansPerSecond);

        // Log battery voltage (brownout monitoring)
        Logger.recordOutput("Drive/BatteryVoltage", RobotController.getBatteryVoltage());

        // Log robot state
        Logger.recordOutput("Drive/Heading", getHeading());
        Logger.recordOutput("Drive/FieldRelative", m_fieldRelative);
        Logger.recordOutput("Drive/SpeedMultiplier", m_speedMultiplier);

        // ═════════════════════════════════════════════════════════════════════
        // SHUFFLEBOARD COMPATIBILITY - Keep critical values for driver display
        // ═════════════════════════════════════════════════════════════════════

        // Driver needs to see these during match
        SmartDashboard.putBoolean("Drive/FieldRelative", m_fieldRelative);
        SmartDashboard.putNumber("Drive/SpeedMultiplier", m_speedMultiplier);
        SmartDashboard.putNumber("Drive/BatteryVoltage", RobotController.getBatteryVoltage());
        SmartDashboard.putBoolean("Drive/BrownoutRisk", m_isBrownedOut);
    }

    // =========================================================================
    // VISION PROCESSING
    // =========================================================================

    /**
     * Processes vision measurements from all cameras and adds safe corrections.
     *
     * <p><b>Safety checks (in order):</b>
     * <ol>
     *   <li><b>Blur check:</b> Reject if spinning > 720°/sec (camera motion blur)
     *   <li><b>Vision timeout:</b> Warn if no tags seen for 2+ seconds
     *   <li><b>Quality filtering:</b> Accept/reject based on tag count and distance
     * </ol>
     *
     * <p><b>Acceptance logic:</b>
     * <ul>
     *   <li><b>Multi-tag (2+ tags):</b> ALWAYS ACCEPT (very reliable)
     *       <ul>
     *         <li>Even if correction is large (> 0.5m), vision is probably right
     *         <li>Use aggressive trust to snap back quickly
     *       </ul>
     *   <li><b>Single tag, close (< 1m drift):</b> ACCEPT (probably good)
     *       <ul>
     *         <li>Normal correction for encoder drift
     *         <li>Use standard trust values from VisionSubsystem
     *       </ul>
     *   <li><b>Single tag, far (> 1m drift):</b> REJECT (probably noise)
     *       <ul>
     *         <li>Could be reflection, wrong tag, or camera glitch
     *         <li>Don't let vision "teleport" robot across field
     *       </ul>
     * </ul>
     *
     * <p><b>⚠️ Tuning:</b> If robot "jumps" to wrong positions, vision might be:
     * <ul>
     *   <li>Seeing reflections (shiny bumpers, glass barriers)
     *   <li>Misconfigured camera Transform3d in RobotContainer
     *   <li>Wrong AprilTag field layout loaded
     * </ul>
     */
    private void processVisionMeasurements() {
        // ═════════════════════════════════════════════════════════════════════
        // SAFETY CHECK #1: Motion Blur
        // ═════════════════════════════════════════════════════════════════════
        // When robot spins fast, camera frames are blurry - can't detect tags accurately
        // Reject vision if spinning too fast (motion blur in camera)
        double angularSpeed = Math.abs(m_gyro.getRate());
        if (angularSpeed > DriveConstants.kMaxAngularVelocityForVisionDegPerSec) {
            String status = "Rejected: Spinning too fast";
            Logger.recordOutput("Vision/Status", status);
            SmartDashboard.putString("Vision/Status", status);
            return;  // Skip vision this frame, use encoders only
        }

        Pose2d currentEstimate = m_poseEstimator.getEstimatedPosition();

        // Get vision estimates from all cameras
        // VisionSubsystem already filtered for quality (ambiguity, distance, etc.)
        List<EstimatedRobotPose> visionEstimates = m_vision.getEstimatedGlobalPoses(currentEstimate);

        // ═════════════════════════════════════════════════════════════════════
        // SAFETY CHECK #2: Vision Timeout Watchdog
        // ═════════════════════════════════════════════════════════════════════
        // Track when we last saw tags - warn if it's been too long
        if (!visionEstimates.isEmpty()) {
            m_lastVisionUpdateTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        }

        double timeSinceLastVision = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - m_lastVisionUpdateTime;
        if (timeSinceLastVision > kVisionTimeoutSeconds && m_lastVisionUpdateTime > 0) {
            String status = "Timeout - No targets for " + String.format("%.1f", timeSinceLastVision) + "s";
            Logger.recordOutput("Vision/Status", status);
            SmartDashboard.putString("Vision/Status", status);
        }

        // ═════════════════════════════════════════════════════════════════════
        // SELECT VISION THRESHOLDS: Classroom Mode vs Competition Mode
        // ═════════════════════════════════════════════════════════════════════
        // Read mode from dashboard - defaults to Competition Mode (safest)
        boolean classroomMode = SmartDashboard.getBoolean("Vision/ClassroomMode", false);

        double largeCorrectionThreshold = classroomMode
            ? DriveConstants.kVisionLargeCorrectionThreshold_Classroom
            : DriveConstants.kVisionLargeCorrectionThreshold_Competition;

        double smallCorrectionThreshold = classroomMode
            ? DriveConstants.kVisionSmallCorrectionThreshold_Classroom
            : DriveConstants.kVisionSmallCorrectionThreshold_Competition;

        // ═════════════════════════════════════════════════════════════════════
        // Process each vision estimate
        // ═════════════════════════════════════════════════════════════════════
        for (EstimatedRobotPose est : visionEstimates) {
            Pose2d estPose = est.estimatedPose.toPose2d();

            // How far is vision estimate from current estimate?
            double distanceDiff = currentEstimate.getTranslation().getDistance(estPose.getTranslation());

            // How many tags did we see?
            boolean multiTag = est.targetsUsed.size() >= 2;

            // ─────────────────────────────────────────────────────────────────
            // DECISION TREE: Should we trust this vision measurement?
            // ─────────────────────────────────────────────────────────────────

            if (multiTag) {
                // ═════════════════════════════════════════════════════════════
                // CASE A: Multi-Tag Detection (HIGH TRUST)
                // ═════════════════════════════════════════════════════════════
                // Seeing 2+ tags simultaneously is VERY reliable
                // Even if we're far from where we thought we were, trust vision!
                // Likely explanation: Our encoders drifted, vision is correcting us

                var stdDevs = m_vision.getEstimationStdDevs(est);

                // If correction is large, trust vision EXTRA to snap back quickly
                if (distanceDiff > largeCorrectionThreshold) {
                    stdDevs = VecBuilder.fill(
                        DriveConstants.kVisionHighTrustStdDevXY,
                        DriveConstants.kVisionHighTrustStdDevXY,
                        DriveConstants.kVisionHighTrustStdDevTheta
                    );  // Low stddev = high trust
                }

                m_poseEstimator.addVisionMeasurement(estPose, est.timestampSeconds, stdDevs);
                String status = "Accepted: Multi-Tag" + (classroomMode ? " [CLASSROOM]" : "");
                Logger.recordOutput("Vision/Status", status);
                SmartDashboard.putString("Vision/Status", status);

            } else if (distanceDiff < smallCorrectionThreshold) {
                // ═════════════════════════════════════════════════════════════
                // CASE B: Single Tag, Small Correction (MEDIUM TRUST)
                // ═════════════════════════════════════════════════════════════
                // Single tag, but close to where we think we are
                // This is normal encoder drift correction - accept it

                var stdDevs = m_vision.getEstimationStdDevs(est);
                m_poseEstimator.addVisionMeasurement(estPose, est.timestampSeconds, stdDevs);
                String status = "Accepted: Single-Tag" + (classroomMode ? " [CLASSROOM]" : "");
                Logger.recordOutput("Vision/Status", status);
                SmartDashboard.putString("Vision/Status", status);

            } else {
                // ═════════════════════════════════════════════════════════════
                // CASE C: Single Tag, Large Correction (REJECT!)
                // ═════════════════════════════════════════════════════════════
                // Single tag saying we're far from where we think we are
                // In Competition Mode: Threshold is 1m (strict safety)
                // In Classroom Mode: Threshold is 5m (relaxed for testing)
                //
                // If rejected, could be:
                //   - Reflection (shiny surface looked like an AprilTag)
                //   - Wrong tag (camera saw tag on different part of field)
                //   - Noise (bad detection)
                //   - Classroom testing: Robot placed far from expected position
                //
                // DON'T trust it - would "teleport" robot
                String status = "Rejected: Single Tag too far (" + String.format("%.2f", distanceDiff) + "m > "
                    + String.format("%.1f", smallCorrectionThreshold) + "m threshold)";
                Logger.recordOutput("Vision/Status", status);
                SmartDashboard.putString("Vision/Status", status);
            }
        }
    }

    // =========================================================================
    // DRIVING METHODS
    // =========================================================================

    /**
     * Drives the robot with given velocities.
     *
     * <p><b>Two driving modes:</b>
     * <ul>
     *   <li><b>Field-relative (fieldRelative=true):</b> Forward always goes "away from driver station"
     *       regardless of robot rotation. This is easier for drivers!
     *   <li><b>Robot-relative (fieldRelative=false):</b> Forward is relative to robot's front.
     *       Used by PathPlanner autonomous.
     * </ul>
     *
     * <p><b>How it works:</b>
     * <ol>
     *   <li>Convert desired speeds to ChassisSpeeds object
     *   <li>Use kinematics to calculate individual module states
     *   <li>Desaturate wheel speeds (prevent exceeding max speed)
     *   <li>Send commands to modules
     * </ol>
     *
     * @param xSpeed Forward velocity in m/s (+ = forward)
     * @param ySpeed Sideways velocity in m/s (+ = left)
     * @param rot Rotation velocity in rad/s (+ = counterclockwise)
     * @param fieldRelative If true, velocities are relative to field (not robot)
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        ChassisSpeeds speeds;

        if (fieldRelative) {
            // Field-relative: Use gyro to transform velocities to robot frame
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d());
        } else {
            // Robot-relative: Use velocities directly
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }

        // Convert chassis speeds to individual module states
        SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

        // Ensure no wheel exceeds max speed (scales all modules proportionally if needed)
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);

        // Command modules to target states
        setModuleStates(states);
    }

    /**
     * Sets target states for all four swerve modules.
     *
     * <p><b>⚠️ Input validation:</b> Checks for null and wrong array size to prevent crashes.
     *
     * <p><b>Array order:</b> [Front-Left, Front-Right, Rear-Left, Rear-Right]
     * This MUST match DriveConstants.kDriveKinematics order!
     *
     * @param desiredStates Array of 4 module states (speed + angle)
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Validate input (prevent crashes from bad data)
        if (desiredStates == null || desiredStates.length != 4) {
            DriverStation.reportError(
                "Invalid module states array - expected 4 states, got " +
                (desiredStates == null ? "null" : desiredStates.length),
                false
            );
            return;
        }

        // Safety check: Desaturate again (in case caller didn't)
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates,
            DriveConstants.kMaxSpeedMetersPerSecond
        );

        // Send commands to modules
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Switches motor brake mode for all swerve modules.
     *
     * <p><b>Brake mode (shouldBrake=true):</b>
     * <ul>
     *   <li>Motors actively resist movement when stopped
     *   <li>Robot "locks in place" - hard to push
     *   <li>Use during matches for precise positioning
     * </ul>
     *
     * <p><b>Coast mode (shouldBrake=false):</b>
     * <ul>
     *   <li>Motors spin freely when stopped
     *   <li>Robot easy to push by hand
     *   <li>Use in pits for easy positioning, reduces motor strain
     * </ul>
     *
     * @param shouldBrake If true, enable brake mode; if false, enable coast mode
     */
    public void setMotorBrake(boolean shouldBrake) {
        m_frontLeft.setBrakeMode(shouldBrake);
        m_frontRight.setBrakeMode(shouldBrake);
        m_rearLeft.setBrakeMode(shouldBrake);
        m_rearRight.setBrakeMode(shouldBrake);
    }

    // =========================================================================
    // POSE ESTIMATION METHODS
    // =========================================================================

    /**
     * Gets the current estimated robot pose from pose estimator.
     *
     * <p><b>This is "the truth"</b> - fused estimate using encoders + gyro + vision.
     *
     * @return Current robot pose (X, Y, rotation) on field
     */
    public Pose2d getCurrentPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets robot pose to a known position.
     *
     * <p><b>When to use:</b>
     * <ul>
     *   <li>At start of autonomous (set to starting position)
     *   <li>When driver presses "reset pose" button mid-match
     *   <li>After force vision reset (snap to AprilTag detection)
     * </ul>
     *
     * <p><b>What it does:</b> Resets BOTH trackers (fused and pure odometry) to given pose.
     *
     * @param pose New robot pose to set
     */
    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
        m_odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    /**
     * Forces pose reset to current vision estimate.
     *
     * <p><b>Use case:</b> Robot is completely lost (odometry drifted badly).
     * Driver presses button to "snap" to nearest AprilTag detection.
     *
     * <p><b>Requirements:</b> At least one AprilTag must be visible!
     *
     * @return True if reset successful (tag was visible), false otherwise
     */
    public boolean forceVisionReset() {
        var visionEstimates = m_vision.getEstimatedGlobalPoses(getCurrentPose());

        if (!visionEstimates.isEmpty()) {
            // Take first vision estimate and snap to it
            EstimatedRobotPose camPose = visionEstimates.get(0);
            resetPose(camPose.estimatedPose.toPose2d());
            return true;
        }

        return false;  // No tags visible - can't reset
    }

    /**
     * Gets current robot rotation from gyro.
     *
     * <p><b>Gyro orientation:</b> This assumes gyro is mounted with:
     * <ul>
     *   <li>Yaw axis pointing up (perpendicular to floor)
     *   <li>Positive rotation = counterclockwise (left) when viewed from above
     * </ul>
     *
     * <p><b>⚠️ IF GYRO MOUNTED DIFFERENTLY:</b> Adjust the negation or use different axis!
     *
     * <p><b>Negation explained:</b> NavX gyro increases angle clockwise, but WPILib
     * uses counterclockwise as positive (standard math convention). We negate to convert.
     *
     * @return Robot rotation as Rotation2d (0° = facing away from driver station)
     */
    public Rotation2d getRotation2d() {
        if (m_gyro.isConnected()) {
            // Negate because NavX is clockwise-positive, WPILib is counterclockwise-positive
            return Rotation2d.fromDegrees(-m_gyro.getAngle());
        }

        // Gyro disconnected - return zero (robot can still drive, but no field-relative)
        return new Rotation2d();
    }

    /**
     * Gets current robot heading in degrees.
     *
     * <p><b>Range:</b> Can exceed 360° (keeps accumulating as robot spins)
     *
     * <p><b>Use getRotation2d() instead</b> if you need normalized angle (0-360°).
     *
     * @return Robot heading in degrees
     */
    public double getHeading() {
        return getRotation2d().getDegrees();
    }

    /**
     * Zeros the gyro heading and resets pose estimator rotation.
     *
     * <p><b>⚠️ IMPORTANT:</b> Robot must be facing AWAY from driver station when this runs!
     *
     * <p><b>What it does:</b>
     * <ol>
     *   <li>Resets gyro to 0° (robot current direction becomes "forward")
     *   <li>Keeps X/Y position unchanged
     *   <li>Sets rotation to 0° in pose estimator
     * </ol>
     *
     * <p><b>Pre-match procedure:</b>
     * <ol>
     *   <li>Place robot facing away from driver station
     *   <li>Run this command
     *   <li>Verify Field/Fused widget shows 0° rotation
     * </ol>
     */
    public void zeroHeading() {
        // Reset gyro hardware to 0°
        m_gyro.reset();

        // Update pose estimator: keep position, reset rotation
        Pose2d current = getCurrentPose();
        resetPose(new Pose2d(current.getTranslation(), new Rotation2d()));
    }

    /**
     * Gets current encoder positions for all 4 modules.
     *
     * <p><b>Array order:</b> [Front-Left, Front-Right, Rear-Left, Rear-Right]
     *
     * <p><b>What's in SwerveModulePosition?</b>
     * <ul>
     *   <li>Distance wheel has traveled (meters)
     *   <li>Current angle of module (Rotation2d)
     * </ul>
     *
     * @return Array of 4 module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        };
    }

    /**
     * Gets current robot velocities in robot-relative frame.
     *
     * <p><b>Frame of reference:</b> Relative to robot (not field!)
     * <ul>
     *   <li>vx = forward/backward velocity
     *   <li>vy = left/right velocity
     *   <li>omega = rotation velocity
     * </ul>
     *
     * <p><b>Used by:</b> PathPlanner autonomous (needs current speed for path following)
     *
     * @return Current robot-relative chassis speeds
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
        );
    }

    // =========================================================================
    // SPEED CONTROL
    // =========================================================================

    /**
     * Sets the speed multiplier for all drive commands.
     *
     * <p><b>How it works:</b> DriveCommands multiply joystick inputs by this value.
     *
     * <p><b>Examples:</b>
     * <ul>
     *   <li>multiplier = 1.0: Full speed (4.8 m/s)
     *   <li>multiplier = 0.5: Half speed (2.4 m/s) - good for precise positioning
     *   <li>multiplier = 0.25: Quarter speed (1.2 m/s) - very slow for fine adjustments
     * </ul>
     *
     * <p><b>Set via:</b> DriveCommands.setSpeed(), toggleSpeed(), fullSpeed(), etc.
     *
     * @param multiplier Speed percentage from 0.0 (stopped) to 1.0 (full speed)
     */
    public void setSpeedMultiplier(double multiplier) {
        // Clamp to 0.0 - 1.0 range
        m_speedMultiplier = edu.wpi.first.math.MathUtil.clamp(multiplier, 0.0, 1.0);

        // Update dashboard so driver can see current speed setting
        SmartDashboard.putNumber("Drive/SpeedMultiplier", m_speedMultiplier);
    }

    /**
     * Gets the current speed multiplier.
     *
     * <p><b>Used by:</b> DriveCommands.joystickDrive() to scale joystick inputs
     *
     * @return Current speed percentage from 0.0 to 1.0
     */
    public double getSpeedMultiplier() {
        return m_speedMultiplier;
    }

    /**
     * Sets field-relative driving mode.
     *
     * <p><b>Field-relative (true):</b> Forward is always "away from driver station"
     * <p><b>Robot-relative (false):</b> Forward is robot's front
     *
     * <p><b>Set via:</b> DriveCommands.toggleFieldRelative()
     *
     * @param fieldRelative true for field-relative, false for robot-relative
     */
    public void setFieldRelative(boolean fieldRelative) {
        m_fieldRelative = fieldRelative;
        SmartDashboard.putBoolean("Drive/FieldRelative", m_fieldRelative);
        String mode = fieldRelative ? "Field-Relative" : "Robot-Relative";
        System.out.println("Drive mode: " + mode);
    }

    /**
     * Gets the current field-relative driving mode.
     *
     * <p><b>Used by:</b> DriveCommands.joystickDrive() to determine driving mode
     *
     * @return true if field-relative, false if robot-relative
     */
    public boolean getFieldRelative() {
        return m_fieldRelative;
    }

    // NOTE: Field visualization now handled by AdvantageScope via logged Pose2d data
    // See updateDashboard() - logs "Drive/Pose" and "Drive/OdometryPose"
    // AdvantageScope will automatically display these on a 3D field widget

    /**
     * Gets the current gyro rotation rate for dashboard telemetry.
     *
     * @return Angular velocity in degrees per second
     */
    public double getGyroRate() {
        return Math.toDegrees(m_gyro.getRate());
    }

    // =========================================================================
    // UTILITY METHODS
    // =========================================================================

    /**
     * Sets swerve modules to form an X-pattern for defensive stability.
     *
     * <p><b>What it does:</b> Rotates modules to 45° angles forming an "X" shape.
     *
     * <p><b>Why?</b> Makes robot VERY hard to push:
     * <ul>
     *   <li>No matter which direction opponent pushes, modules resist
     *   <li>Works like "parking brake" for swerve drive
     *   <li>Much more stable than wheels pointing straight
     * </ul>
     *
     * <p><b>When to use:</b>
     * <ul>
     *   <li>Defense: Hold position against opponent pushes
     *   <li>End of match: Lock in place on platform/ramp
     *   <li>Disabled: Prevent robot from rolling on sloped floor
     * </ul>
     *
     * <p><b>Module angles:</b>
     * <pre>
     *  ╲     ╱     Front of robot
     *   FL  FR
     *   RL  RR
     *  ╱     ╲
     * </pre>
     */
    public void setX() {
        setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(DriveConstants.kXStanceAngleDegrees)),   // FL
            new SwerveModuleState(0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-DriveConstants.kXStanceAngleDegrees)),  // FR
            new SwerveModuleState(0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-DriveConstants.kXStanceAngleDegrees)),  // RL
            new SwerveModuleState(0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(DriveConstants.kXStanceAngleDegrees))    // RR
        });
    }

    /**
     * Sets all swerve modules to point straight ahead (0 degrees).
     *
     * <p><b>Purpose:</b> Aligns wheels forward for pit crew inspection and pre-match setup.
     *
     * <p><b>Why this matters:</b>
     * <ul>
     *   <li>Easier for pit crew to push robot in straight line when disabled
     *   <li>Visual confirmation that all modules are mechanically aligned correctly
     *   <li>Standard pre-match preparation step
     *   <li>Makes it obvious if any module has mechanical issues
     * </ul>
     *
     * <p><b>When to use:</b>
     * <ul>
     *   <li>Before each match: Pit crew clicks button before queueing
     *   <li>After maintenance: Verify all modules respond correctly
     *   <li>During inspection: Show judges that modules work properly
     * </ul>
     *
     * <p><b>Module angles:</b>
     * <pre>
     *  ↑  ↑     Front of robot
     *  FL FR
     *  RL RR
     *  ↑  ↑
     * </pre>
     */
    public void setStraightAhead() {
        setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0)),  // FL
            new SwerveModuleState(0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0)),  // FR
            new SwerveModuleState(0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0)),  // RL
            new SwerveModuleState(0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0))   // RR
        });
    }

    /**
     * Calculates distance from robot's current position to a target pose.
     *
     * <p><b>Helper for:</b> Game-specific commands (ReefscapeCommands)
     * <ul>
     *   <li>Finding nearest scoring location
     *   <li>Checking if robot is close enough to score
     *   <li>Deciding which path to take
     * </ul>
     *
     * <p><b>Note:</b> Ignores rotation - only compares X/Y positions!
     *
     * @param target The target pose on the field
     * @return Straight-line distance to target in meters
     */
    public double getDistanceToTarget(Pose2d target) {
        return getCurrentPose().getTranslation().getDistance(target.getTranslation());
    }

    // =========================================================================
    // PATHPLANNER CONFIGURATION
    // =========================================================================

    /**
     * Configures PathPlanner for autonomous path following.
     *
     * <p><b>What is PathPlanner?</b> Software for creating autonomous driving paths.
     * Allows drawing paths on field map, robot follows them accurately.
     *
     * <p><b>Configuration bindings:</b>
     * <ul>
     *   <li>Pose supplier: Where robot currently is (getCurrentPose)
     *   <li>Pose reset: How to set starting position (resetPose)
     *   <li>Speed getter: Current robot velocity (getRobotRelativeSpeeds)
     *   <li>Speed setter: How to drive robot (drive method in robot-relative mode)
     *   <li>PID controllers: From AutoConstants (translation and rotation)
     *   <li>Alliance flipper: Automatically mirror paths for red alliance
     * </ul>
     *
     * <p><b>⚠️ DO NOT MODIFY</b> unless changing auto path following behavior!
     */
    private void configurePathPlanner() {
        RobotConfig config;

        try {
            // Load robot physical config from PathPlanner GUI
            // (wheelbase, mass, bumper size, motor characteristics, etc.)
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config: " + e.getMessage(), true);
            return;  // Can't configure - auto paths won't work
        }

        // Configure AutoBuilder with method references
        AutoBuilder.configure(
            this::getCurrentPose,              // How to get current robot pose
            this::resetPose,                   // How to reset pose at start of path
            this::getRobotRelativeSpeeds,      // How to get current speeds

            // How to drive robot (robot-relative mode!)
            (speeds, feedforwards) -> drive(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                false  // Robot-relative (PathPlanner provides already-transformed speeds)
            ),

            // PID controllers for path following
            new PPHolonomicDriveController(
                DriveConstants.kTranslationPID,  // X/Y position PID
                DriveConstants.kRotationPID      // Rotation PID
            ),

            config,  // Robot physical characteristics

            // Alliance flipping: Automatically mirror paths for red alliance
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == Alliance.Red;
            },

            this  // Subsystem requirement
        );
    }
}
