package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;

/**
 * ═══════════════════════════════════════════════════════════════════════════
 *                           VISION SUBSYSTEM
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * ✅ YEAR-TO-YEAR REUSABLE - This class should NOT need changes between seasons!
 *
 * Purpose: Provides AprilTag-based robot localization using PhotonVision cameras.
 * Works with DriveSubsystem to improve pose estimation accuracy.
 *
 * Features:
 *   ✅ Multi-camera support (automatically handles 1+ cameras)
 *   ✅ Safety "Firewall" - Catches exceptions to prevent robot crashes
 *   ✅ Kill switch - Can be disabled via Dashboard "Vision/Enabled"
 *   ✅ Robust filtering - Rejects low-quality/ambiguous detections
 *   ✅ Implements VisionProvider interface for dependency injection
 *
 * ⚠️ WHAT TO UPDATE EACH YEAR:
 *   1. In RobotContainer.java: Update camera name and Transform3d (position/rotation)
 *   2. In Constants.VisionConstants: Tune MAX_SINGLE_TAG_DIST and MAX_TAG_AMBIGUITY if needed
 *   3. Field Layout: Ensure RobotContainer loads correct year's AprilTag layout
 *
 * ⚠️ DO NOT MODIFY THIS FILE unless you're changing core vision logic!
 *
 * How it works:
 *   1. Cameras detect AprilTags on the field
 *   2. PhotonVision calculates where the robot is based on tag positions
 *   3. DriveSubsystem requests poses via getEstimatedGlobalPoses(Pose2d)
 *   4. DriveSubsystem fuses vision data with wheel odometry for accurate positioning
 *
 * Dashboard Keys (logged to AdvantageKit + SmartDashboard):
 *   - Vision/Enabled - Toggle vision system on/off
 *   - Vision/[CameraName]/Connected - Camera connection status
 *   - Vision/[CameraName]/TargetCount - How many tags camera sees
 *   - Vision/[CameraName]/LatencyMs - Camera processing delay
 *   - Vision/[CameraName]/EstimatedPose2d - Vision pose (view in AdvantageScope)
 *
 * Related files:
 *   - VisionProvider: Interface this implements
 *   - NoVisionProvider: Fallback when cameras unavailable
 *   - DriveSubsystem: Where vision data is consumed
 */
public class VisionSubsystem extends SubsystemBase implements VisionProvider {

    // =========================================================================
    // VISION QUALITY FILTERS
    // ⚠️ TUNE THESE AT START OF SEASON IF VISION IS UNRELIABLE
    // =========================================================================

    // ═══════════════════════════════════════════════════════════════════════
    // VISION QUALITY FILTERS - Values from Constants.VisionConstants
    // ═══════════════════════════════════════════════════════════════════════

    /** Maximum distance for single-tag trust - From VisionConstants.kMaxVisionDistanceMeters */
    private static final double MAX_SINGLE_TAG_DIST = VisionConstants.kMaxVisionDistanceMeters;

    /** Maximum ambiguity for single-tag acceptance - From VisionConstants.kMaxAmbiguity */
    private static final double MAX_TAG_AMBIGUITY = VisionConstants.kMaxAmbiguity;

    // =========================================================================
    // CAMERA CONFIGURATION HELPER CLASS
    // =========================================================================

    /**
     * Stores camera name and physical position on robot.
     *
     * ⚠️ UPDATE IN RobotContainer.java, NOT HERE!
     *
     * Example from RobotContainer:
     *
     * new VisionSubsystem.CameraConfig(
     *     "photonvision",  // Name in PhotonVision UI
     *     new Transform3d(
     *         new Translation3d(0.3, 0, 0.5),  // 0.3m forward, 0.5m up
     *         new Rotation3d(0, Math.toRadians(-30), 0)  // Tilted 30° down
     *     )
     * )
     */
    public static class CameraConfig {
        /** Camera name as configured in PhotonVision dashboard */
        public final String name;

        /** Transform from robot center to camera (position + rotation) */
        public final Transform3d robotToCamera;

        public CameraConfig(String name, Transform3d robotToCamera) {
            this.name = name;
            this.robotToCamera = robotToCamera;
        }
    }

    // =========================================================================
    // SUBSYSTEM STATE
    // =========================================================================

    /** Pose estimators (one per camera) - calculate robot position from AprilTags */
    private final List<PhotonPoseEstimator> m_estimators = new ArrayList<>();

    /** Camera objects - communicate with PhotonVision */
    private final List<PhotonCamera> m_cameras = new ArrayList<>();

    /** AprilTag field layout - defines where tags are on the field */
    private final AprilTagFieldLayout m_fieldLayout;

    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================

    /**
     * Creates vision subsystem with one or more cameras.
     *
     * <p><b>⚠️ DO NOT CALL DIRECTLY!</b> This is called from RobotContainer.
     *
     * @param fieldLayout AprilTag positions for current game (loaded from WPILib)
     * @param configs One or more camera configurations (name + position)
     *
     * @see frc.robot.RobotContainer#createVisionProvider() Where this is called
     */
    public VisionSubsystem(AprilTagFieldLayout fieldLayout, CameraConfig... configs) {
        m_fieldLayout = fieldLayout;

        // Initialize vision kill switch (default = enabled)
        SmartDashboard.setDefaultBoolean("Vision/Enabled", true);

        // SAFETY CHECK: If no field layout provided, vision can't work
        // We report the error but DON'T crash - robot can still drive with encoders only
        if (m_fieldLayout == null) {
            DriverStation.reportError(
                "VisionSubsystem: Field Layout is null! Vision will be disabled.",
                true
            );
            // Continue initialization so code structure remains valid
            // Update loop will safely bail out when called
        }

        // Initialize each camera
        for (CameraConfig config : configs) {
            try {
                // Create camera connection
                PhotonCamera camera = new PhotonCamera(config.name);

                // Only create pose estimator if we have a field layout
                if (m_fieldLayout != null) {
                    PhotonPoseEstimator estimator = new PhotonPoseEstimator(
                        m_fieldLayout,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,  // Use multi-tag when possible
                        config.robotToCamera
                    );

                    // Fallback strategy: If only one tag visible, use "cleanest" detection
                    estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                    m_estimators.add(estimator);
                }

                m_cameras.add(camera);

            } catch (Exception e) {
                // Camera failed to initialize - report but continue
                // Robot can still work with remaining cameras (or no vision at all)
                DriverStation.reportError(
                    "Vision: Failed to init camera " + config.name + ": " + e.getMessage(),
                    true
                );
            }
        }
    }

    // =========================================================================
    // VISION PROVIDER INTERFACE METHODS
    // These are called by DriveSubsystem to get pose estimates
    // =========================================================================

    /**
     * Gets all valid robot pose estimates from all cameras.
     *
     * <p><b>How this works:</b>
     * <ol>
     *   <li>Check if vision is enabled (Dashboard kill switch)
     *   <li>Loop through each camera
     *   <li>Get pose estimate from PhotonVision
     *   <li>Filter out low-quality detections (ambiguous single tags, too far, etc.)
     *   <li>Return list of good estimates
     * </ol>
     *
     * <p><b>Safety features:</b>
     * <ul>
     *   <li>✅ Returns empty list if vision disabled (not null!)
     *   <li>✅ Catches all exceptions to prevent robot crash
     *   <li>✅ Filters bad detections before returning
     *   <li>✅ Works even if cameras disconnect mid-match
     * </ul>
     *
     * <p><b>Called by:</b> DriveSubsystem.periodic() every 20ms
     *
     * @param prevEstimatedRobotPose Current best guess of robot position (from encoders + previous vision)
     *                                Used by PhotonVision to resolve ambiguous detections
     * @return List of valid pose estimates (empty if no tags seen or vision disabled)
     */
    @Override
    public List<EstimatedRobotPose> getEstimatedGlobalPoses(Pose2d prevEstimatedRobotPose) {
        List<EstimatedRobotPose> validEstimates = new ArrayList<>();

        // For dashboard visualization - shows where vision thinks robot is
        Pose2d debugPose = new Pose2d(-10, -10, new Rotation2d()); // Default: off-field
        boolean anyTargetSeen = false;

        // ═════════════════════════════════════════════════════════════════════
        // SAFETY CHECK #1: Kill Switch
        // ═════════════════════════════════════════════════════════════════════
        // Allows driver/coach to disable vision if it's misbehaving during match
        if (!SmartDashboard.getBoolean("Vision/Enabled", true)) {
            return validEstimates; // Return empty - DriveSubsystem uses encoders only
        }

        // ═════════════════════════════════════════════════════════════════════
        // SAFETY CHECK #2: Field Layout
        // ═════════════════════════════════════════════════════════════════════
        // Can't calculate poses without knowing where AprilTags are!
        if (m_fieldLayout == null) {
            return validEstimates;
        }

        // ═════════════════════════════════════════════════════════════════════
        // SAFETY CHECK #3: Exception Firewall
        // ═════════════════════════════════════════════════════════════════════
        // Catch ANY exception from PhotonVision library to prevent robot crash
        try {
            // Process each camera
            for (int i = 0; i < m_estimators.size(); i++) {
                PhotonPoseEstimator estimator = m_estimators.get(i);
                PhotonCamera camera = m_cameras.get(i);

                // Tell PhotonVision where we think we are
                // This helps resolve ambiguous tag orientations
                estimator.setReferencePose(prevEstimatedRobotPose);

                // ─────────────────────────────────────────────────────────────
                // EXTERNAL LIBRARY CALL - This is where PhotonVision runs
                // ─────────────────────────────────────────────────────────────
                // Get all unread results and use the latest one
                var results = camera.getAllUnreadResults();
                if (results.isEmpty()) {
                    // Log that camera is connected but no new results
                    logCameraStatus(camera, null, null);
                    continue; // No new results, skip to next camera
                }
                var pipelineResult = results.get(results.size() - 1); // Get most recent
                Optional<EstimatedRobotPose> result = estimator.update(pipelineResult);
                // ─────────────────────────────────────────────────────────────

                // Log camera status to dashboard for debugging (pass result AND pipelineResult)
                logCameraStatus(camera, pipelineResult, result.orElse(null));

                // Did this camera see any tags?
                if (result.isPresent()) {
                    EstimatedRobotPose estimate = result.get();
                    var targets = estimate.targetsUsed;

                    // ═════════════════════════════════════════════════════════
                    // QUALITY FILTER: Reject ambiguous single-tag detections
                    // ═════════════════════════════════════════════════════════
                    // Multi-tag detections are always trusted (very accurate)
                    // Single-tag detections are checked for ambiguity score
                    if (targets.size() == 1) {
                        double ambiguity = targets.get(0).getPoseAmbiguity();

                        // Skip this detection if ambiguity too high
                        if (ambiguity > MAX_TAG_AMBIGUITY) {
                            continue; // Move to next camera
                        }
                    }

                    // This estimate passed all filters - add to results
                    validEstimates.add(estimate);

                    // Update dashboard visualization (just use first valid pose)
                    if (!anyTargetSeen) {
                        debugPose = estimate.estimatedPose.toPose2d();
                        anyTargetSeen = true;
                    }
                }
            }

        } catch (Exception e) {
            // ═════════════════════════════════════════════════════════════════
            // CRITICAL ERROR HANDLER
            // ═════════════════════════════════════════════════════════════════
            // If we get here, PhotonVision crashed or camera disconnected mid-update
            // We catch the exception so the robot loop doesn't die
            // Robot continues with encoder-only odometry
            DriverStation.reportError(
                "CRITICAL: Vision System Exception! Ignoring this frame. " + e.getMessage(),
                false  // Don't print stack trace (too spammy)
            );
            return validEstimates; // Return whatever we collected before crash
        }

        // Update dashboard field widget
        m_fieldVision.setRobotPose(debugPose);

        return validEstimates;
    }

    /**
     * Calculates "trust levels" (standard deviations) for a vision estimate.
     *
     * <p><b>What are standard deviations?</b> They tell DriveSubsystem how much to
     * trust this vision measurement vs. encoder odometry:
     * <ul>
     *   <li><b>Lower numbers</b> = Higher trust (vision measurement very accurate)
     *   <li><b>Higher numbers</b> = Lower trust (encoders probably more accurate)
     * </ul>
     *
     * <p><b>Trust rules:</b>
     * <ol>
     *   <li><b>Multi-tag detection:</b> HIGH TRUST (0.5m, 10°) - Very accurate!
     *   <li><b>Single tag, close range:</b> MEDIUM TRUST (0.9m, 30°) - Pretty good
     *   <li><b>Single tag, far range:</b> LOW TRUST (increases with distance²)
     *   <li><b>Single tag, too far:</b> NO TRUST (100m, 100°) - Reject it
     * </ol>
     *
     * <p><b>Why distance matters:</b> Far-away tags are small in the camera image,
     * making them harder to detect accurately. Close tags fill more pixels = better accuracy.
     *
     * <p>⚠️ These trust values are tuned for typical FRC fields. If vision seems
     * too "jumpy" or "sluggish", adjust the VecBuilder values below.
     *
     * @param estimatedPose The vision pose estimate to evaluate
     * @return Matrix of standard deviations [x_stddev, y_stddev, theta_stddev]
     */
    @Override
    public Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose estimatedPose) {
        var targets = estimatedPose.targetsUsed;
        int numTags = targets.size();

        // Calculate average distance to all tags in this estimate
        double avgDist = 0;
        for (PhotonTrackedTarget tgt : targets) {
            var tagPose = m_fieldLayout.getTagPose(tgt.getFiducialId());
            if (tagPose.isPresent()) {
                avgDist += tagPose.get().toPose2d().getTranslation().getDistance(
                    estimatedPose.estimatedPose.toPose2d().getTranslation()
                );
            }
        }
        if (numTags > 0) {
            avgDist /= numTags;
        }

        // Calculate trust based on number of tags and distance
        Matrix<N3, N1> estStdDevs;

        if (numTags >= 2) {
            // ═════════════════════════════════════════════════════════════════
            // MULTI-TAG DETECTION: Very high trust
            // ═════════════════════════════════════════════════════════════════
            // Seeing 2+ tags simultaneously is VERY accurate because:
            // - PhotonVision can triangulate between tags
            // - Ambiguity is eliminated
            // - Errors cancel out
            //
            // Values from Constants.VisionConstants
            estStdDevs = VecBuilder.fill(
                VisionConstants.kMultiTagHighTrustStdDevXY,
                VisionConstants.kMultiTagHighTrustStdDevXY,
                Math.toRadians(VisionConstants.kMultiTagHighTrustStdDevThetaDegrees)
            );

        } else {
            // ═════════════════════════════════════════════════════════════════
            // SINGLE-TAG DETECTION: Trust scales with distance
            // ═════════════════════════════════════════════════════════════════

            if (avgDist > MAX_SINGLE_TAG_DIST) {
                // Tag is too far - don't trust it at all
                // Standard dev = kVisionNoTrustStdDev means "ignore this, encoders are better"
                estStdDevs = VecBuilder.fill(
                    DriveConstants.kVisionNoTrustStdDev,
                    DriveConstants.kVisionNoTrustStdDev,
                    DriveConstants.kVisionNoTrustStdDev
                );

            } else {
                // Tag is within range - trust decreases with distance
                // Trust scale formula from VisionConstants.kVisionTrustScaleDenominator
                double trustScale = 1 + (avgDist * avgDist / VisionConstants.kVisionTrustScaleDenominator);

                estStdDevs = VecBuilder.fill(
                    VisionConstants.kSingleTagBaseTrustStdDevXY * trustScale,
                    VisionConstants.kSingleTagBaseTrustStdDevXY * trustScale,
                    Math.toRadians(VisionConstants.kSingleTagBaseTrustStdDevThetaDegrees) * trustScale
                );
            }
        }

        return estStdDevs;
    }

    // =========================================================================
    // HELPER METHODS
    // =========================================================================

    /**
     * Logs comprehensive camera status and detection details to SmartDashboard.
     *
     * <p><b>Dashboard keys created:</b>
     * <ul>
     *   <li>{@code Vision/[CameraName]/Connected} - Is camera responding?
     *   <li>{@code Vision/[CameraName]/TargetCount} - How many tags detected?
     *   <li>{@code Vision/[CameraName]/LatencyMs} - Camera processing delay
     *   <li>{@code Vision/[CameraName]/TagIDs} - Comma-separated list of detected tag IDs
     *   <li>{@code Vision/[CameraName]/EstimatedPose} - Robot pose computed from vision
     *   <li>{@code Vision/[CameraName]/AvgTagDistance} - Average distance to detected tags
     * </ul>
     *
     * <p><b>Use this for:</b>
     * <ul>
     *   <li>Verifying camera is connected before match
     *   <li>Checking which specific tags are being detected
     *   <li>Diagnosing vision latency issues
     *   <li>Validating camera positioning and range
     * </ul>
     *
     * @param camera The camera to check
     * @param pipelineResult The raw pipeline result (may be null if no results)
     * @param estimatedPose The computed robot pose (may be null if no valid estimate)
     */
    private void logCameraStatus(PhotonCamera camera, org.photonvision.PhotonPipelineResult pipelineResult,
                                  EstimatedRobotPose estimatedPose) {
        boolean connected = camera.isConnected();
        String baseKey = "Vision/" + camera.getName();

        // Always log connection status
        Logger.recordOutput(baseKey + "/Connected", connected);
        SmartDashboard.putBoolean(baseKey + "/Connected", connected);

        // If no pipeline result, camera is disconnected or no new data
        if (pipelineResult == null) {
            Logger.recordOutput(baseKey + "/TargetCount", 0);
            Logger.recordOutput(baseKey + "/TagIDs", "None");
            Logger.recordOutput(baseKey + "/EstimatedPose", "N/A");
            Logger.recordOutput(baseKey + "/AvgTagDistance", 0.0);
            SmartDashboard.putNumber(baseKey + "/TargetCount", 0);
            SmartDashboard.putString(baseKey + "/TagIDs", "None");
            SmartDashboard.putString(baseKey + "/EstimatedPose", "N/A");
            SmartDashboard.putNumber(baseKey + "/AvgTagDistance", 0.0);
            return;
        }

        // Log latency
        double latencyMs = pipelineResult.getLatencyMillis();
        Logger.recordOutput(baseKey + "/LatencyMs", latencyMs);
        SmartDashboard.putNumber(baseKey + "/LatencyMs", latencyMs);

        // Get detected targets
        var targets = pipelineResult.getTargets();
        Logger.recordOutput(baseKey + "/TargetCount", targets.size());
        SmartDashboard.putNumber(baseKey + "/TargetCount", targets.size());

        // Log individual tag IDs and build comma-separated list
        if (targets.isEmpty()) {
            Logger.recordOutput(baseKey + "/TagIDs", "None");
            Logger.recordOutput(baseKey + "/EstimatedPose", "N/A");
            Logger.recordOutput(baseKey + "/AvgTagDistance", 0.0);
            SmartDashboard.putString(baseKey + "/TagIDs", "None");
            SmartDashboard.putString(baseKey + "/EstimatedPose", "N/A");
            SmartDashboard.putNumber(baseKey + "/AvgTagDistance", 0.0);
        } else {
            // Build comma-separated list of tag IDs with their distances
            StringBuilder tagInfo = new StringBuilder();
            double totalDistance = 0.0;
            int validDistances = 0;

            for (int i = 0; i < targets.size(); i++) {
                PhotonTrackedTarget target = targets.get(i);
                int tagId = target.getFiducialId();

                // Get tag pose from field layout to calculate distance
                var tagPose = m_fieldLayout.getTagPose(tagId);

                if (i > 0) {
                    tagInfo.append(", ");
                }

                tagInfo.append("ID").append(tagId);

                // Add distance information if we have the tag pose and estimated robot pose
                if (tagPose.isPresent() && estimatedPose != null) {
                    double distance = tagPose.get().toPose2d().getTranslation().getDistance(
                        estimatedPose.estimatedPose.toPose2d().getTranslation()
                    );
                    tagInfo.append(String.format("(%.2fm)", distance));
                    totalDistance += distance;
                    validDistances++;
                } else {
                    // Fallback: use camera-to-target transform distance
                    var bestTransform = target.getBestCameraToTarget();
                    double distance = Math.sqrt(
                        bestTransform.getX() * bestTransform.getX() +
                        bestTransform.getY() * bestTransform.getY() +
                        bestTransform.getZ() * bestTransform.getZ()
                    );
                    tagInfo.append(String.format("(~%.2fm)", distance));
                }

                // Add ambiguity info for debugging
                double ambiguity = target.getPoseAmbiguity();
                if (ambiguity > MAX_TAG_AMBIGUITY) {
                    tagInfo.append("[HIGH_AMB:").append(String.format("%.2f", ambiguity)).append("]");
                }
            }

            Logger.recordOutput(baseKey + "/TagIDs", tagInfo.toString());
            SmartDashboard.putString(baseKey + "/TagIDs", tagInfo.toString());

            // Log average distance
            if (validDistances > 0) {
                double avgDistance = totalDistance / validDistances;
                Logger.recordOutput(baseKey + "/AvgTagDistance", avgDistance);
                SmartDashboard.putNumber(baseKey + "/AvgTagDistance", avgDistance);
            }

            // Log estimated pose if available
            if (estimatedPose != null) {
                Pose2d pose = estimatedPose.estimatedPose.toPose2d();

                // Log pose as Pose2d struct (AdvantageScope will display in 3D)
                Logger.recordOutput(baseKey + "/EstimatedPose2d", pose);

                // Also log human-readable string for Shuffleboard
                String poseStr = String.format("(%.2f, %.2f, %.1f°)",
                    pose.getX(),
                    pose.getY(),
                    pose.getRotation().getDegrees()
                );
                Logger.recordOutput(baseKey + "/EstimatedPose", poseStr);
                SmartDashboard.putString(baseKey + "/EstimatedPose", poseStr);

                // Log individual components for AdvantageKit graphing
                Logger.recordOutput(baseKey + "/PoseX", pose.getX());
                Logger.recordOutput(baseKey + "/PoseY", pose.getY());
                Logger.recordOutput(baseKey + "/PoseRotation", pose.getRotation().getDegrees());

                // Keep SmartDashboard for Shuffleboard compatibility
                SmartDashboard.putNumber(baseKey + "/PoseX", pose.getX());
                SmartDashboard.putNumber(baseKey + "/PoseY", pose.getY());
                SmartDashboard.putNumber(baseKey + "/PoseRotation", pose.getRotation().getDegrees());
            } else {
                Logger.recordOutput(baseKey + "/EstimatedPose", "Rejected");
                SmartDashboard.putString(baseKey + "/EstimatedPose", "Rejected");
            }
        }
    }

    // =========================================================================
    // SUBSYSTEM PERIODIC
    // =========================================================================

    /**
     * Called every 20ms by the command scheduler.
     *
     * <p><b>Why this is here:</b> When robot is disabled, DriveSubsystem doesn't
     * call {@link #getEstimatedGlobalPoses}, which means cameras stop updating.
     * This periodic call keeps cameras "alive" during disabled state so dashboard
     * shows live camera feed and tag detections.
     *
     * <p>This helps drivers/programmers verify camera is working before match starts.
     */
    @Override
    public void periodic() {
        // When robot is disabled, force a vision update
        // Pass dummy pose (0,0) just to trigger camera reads
        if (DriverStation.isDisabled()) {
            getEstimatedGlobalPoses(new Pose2d());
        }
    }
}
