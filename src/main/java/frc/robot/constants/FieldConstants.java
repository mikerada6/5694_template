package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Field-specific constants for 2025 Reefscape game.
 *
 * ⚠️ REPLACE THESE EACH YEAR with actual field measurements!
 * These are examples - measure actual positions during calibration.
 */
public final class FieldConstants {

  // ═══════════════════════════════════════════════════════════════════════
  // FIELD DIMENSIONS
  // ═══════════════════════════════════════════════════════════════════════

  /** Field length (meters) - Blue to Red alliance walls. */
  public static final double kFieldLength = 16.54175;

  /** Field width (meters) - Driver station to opposite wall. */
  public static final double kFieldWidth = 8.0137;

  // ═══════════════════════════════════════════════════════════════════════
  // BLUE ALLIANCE SCORING POSITIONS
  // ═══════════════════════════════════════════════════════════════════════

  /** Blue reef left scoring position. */
  public static final Pose2d kBlueReefLeft = new Pose2d(2.0, 5.5, Rotation2d.fromDegrees(0));

  /** Blue reef center scoring position. */
  public static final Pose2d kBlueReefCenter = new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0));

  /** Blue reef right scoring position. */
  public static final Pose2d kBlueReefRight = new Pose2d(2.0, 2.5, Rotation2d.fromDegrees(0));

  /** Blue processor scoring position. */
  public static final Pose2d kBlueProcessor = new Pose2d(1.5, 7.0, Rotation2d.fromDegrees(-90));

  // ═══════════════════════════════════════════════════════════════════════
  // RED ALLIANCE SCORING POSITIONS
  // ═══════════════════════════════════════════════════════════════════════

  /** Red reef left scoring position (mirrored from blue). */
  public static final Pose2d kRedReefLeft = new Pose2d(14.5, 5.5, Rotation2d.fromDegrees(180));

  /** Red reef center scoring position (mirrored from blue). */
  public static final Pose2d kRedReefCenter = new Pose2d(14.5, 4.0, Rotation2d.fromDegrees(180));

  /** Red reef right scoring position (mirrored from blue). */
  public static final Pose2d kRedReefRight = new Pose2d(14.5, 2.5, Rotation2d.fromDegrees(180));

  /** Red processor scoring position (mirrored from blue). */
  public static final Pose2d kRedProcessor = new Pose2d(15.0, 7.0, Rotation2d.fromDegrees(90));

  // ═══════════════════════════════════════════════════════════════════════
  // SCORING DISTANCE OFFSETS
  // ═══════════════════════════════════════════════════════════════════════

  /** Distance to stop from reef target (meters). Robot positions 0.5m away before scoring. */
  public static final double kReefScoringDistance = 0.5;

  /** Distance to stop from processor target (meters). */
  public static final double kProcessorScoringDistance = 0.4;

  private FieldConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
