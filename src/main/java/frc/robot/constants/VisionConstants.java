package frc.robot.constants;

/**
 * Vision system constants: camera configuration and trust parameters.
 *
 * Change these when:
 * - Installing/moving cameras (position, rotation, name)
 * - Tuning vision trust (distance limits, ambiguity, std devs)
 */
public final class VisionConstants {

  // ═══════════════════════════════════════════════════════════════════════
  // CAMERA IDENTIFICATION
  // ═══════════════════════════════════════════════════════════════════════

  /**
   * Camera name in PhotonVision.
   *
   * Must match name in PhotonVision UI (http://photonvision.local:5800).
   * Change this if you rename the camera or add multiple cameras.
   */
  public static final String kCameraName = "YOUR_CAMERA_NAME";

  // ═══════════════════════════════════════════════════════════════════════
  // CAMERA POSITION (Robot-to-Camera Transform)
  // ═══════════════════════════════════════════════════════════════════════

  /**
   * Camera X offset from robot center (meters).
   *
   * + = forward, - = backward
   * Measure from robot center (intersection of wheel diagonals).
   */
  public static final double kCameraToRobotX = 0.0;

  /**
   * Camera Y offset from robot center (meters).
   *
   * + = left, - = right
   * Measure from robot center.
   */
  public static final double kCameraToRobotY = 0.0;

  /**
   * Camera Z offset from robot center (meters).
   *
   * + = up, - = down
   * Measure from robot center.
   */
  public static final double kCameraToRobotZ = 0.0;

  /**
   * Camera pitch angle (radians).
   *
   * Tilt up/down:
   * - Negative = tilted down (common for AprilTags on floor)
   * - 0 = level with ground
   * - Positive = tilted up
   *
   * Example: -0.52 rad = -30° tilt down
   */
  public static final double kCameraPitchRadians = 0.0;

  /**
   * Camera yaw angle (radians).
   *
   * Pan left/right:
   * - 0 = facing forward (most common)
   * - 1.57 = facing left (90°)
   * - -1.57 = facing right (-90°)
   */
  public static final double kCameraYawRadians = 0.0;

  /**
   * Camera roll angle (radians).
   *
   * Usually 0.0 unless camera is mounted sideways.
   */
  public static final double kCameraRollRadians = 0.0;

  // ═══════════════════════════════════════════════════════════════════════
  // VISION QUALITY FILTERS
  // ═══════════════════════════════════════════════════════════════════════

  /**
   * Max distance for single-tag detections (meters).
   *
   * Reject single tags farther than this. Multi-tag always accepted.
   * Increase if field is large, decrease if getting bad detections.
   */
  public static final double kMaxVisionDistanceMeters = 4.0;

  /**
   * Max ambiguity for AprilTag detections (0.0 to 1.0).
   *
   * Ambiguity measures detection quality:
   * - 0.0 = perfect, unambiguous
   * - 0.2 = good (our threshold)
   * - 0.5+ = questionable, likely bad
   *
   * Lower = stricter filtering, fewer but better detections.
   */
  public static final double kMaxAmbiguity = 0.2;

  // ═══════════════════════════════════════════════════════════════════════
  // VISION TRUST (Standard Deviations)
  // ═══════════════════════════════════════════════════════════════════════

  /**
   * Multi-tag high trust std dev - X/Y position (meters).
   *
   * When 2+ tags detected, trust vision highly.
   * Lower = higher trust.
   */
  public static final double kMultiTagHighTrustStdDevXY = 0.5;

  /**
   * Multi-tag high trust std dev - Rotation (degrees).
   *
   * Converted to radians in code via Math.toRadians().
   */
  public static final double kMultiTagHighTrustStdDevThetaDegrees = 10.0;

  /**
   * Single-tag base trust std dev - X/Y position (meters).
   *
   * Base uncertainty before distance scaling.
   * Multiplied by distance-based trust scale in VisionSubsystem.
   */
  public static final double kSingleTagBaseTrustStdDevXY = 0.9;

  /**
   * Single-tag base trust std dev - Rotation (degrees).
   *
   * Converted to radians in code via Math.toRadians().
   */
  public static final double kSingleTagBaseTrustStdDevThetaDegrees = 30.0;

  /**
   * Trust scale formula denominator.
   *
   * Used in: trustScale = 1 + (distance² / denominator)
   *
   * Current value: 30
   * - 1m away: scale = 1.03 (very trustworthy)
   * - 2m away: scale = 1.13 (pretty good)
   * - 3m away: scale = 1.30 (okay)
   */
  public static final double kVisionTrustScaleDenominator = 30.0;

  private VisionConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
