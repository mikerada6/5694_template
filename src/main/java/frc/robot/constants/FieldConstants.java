package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Field-specific constants for [CURRENT SEASON GAME].
 *
 * ⚠️ REPLACE THESE EACH YEAR with actual field measurements!
 * These are TEMPLATE EXAMPLES - measure actual positions during field calibration.
 *
 * <p><b>How to update for new season:</b>
 * <ol>
 *   <li>Review field drawings from FRC game manual
 *   <li>Identify key scoring locations and their coordinates
 *   <li>Replace example positions below with actual measurements
 *   <li>Rename constants to match game terminology (e.g., kBlueSpeaker, kRedAmp)
 *   <li>Update distance offsets based on robot dimensions
 * </ol>
 */
public final class FieldConstants {

  // ═══════════════════════════════════════════════════════════════════════
  // FIELD DIMENSIONS (Standard for all FRC fields)
  // ═══════════════════════════════════════════════════════════════════════

  /** Field length (meters) - Blue alliance wall to Red alliance wall. */
  public static final double kFieldLength = 16.54175;

  /** Field width (meters) - Driver station wall to opposite wall. */
  public static final double kFieldWidth = 8.0137;

  // ═══════════════════════════════════════════════════════════════════════
  // BLUE ALLIANCE SCORING POSITIONS
  // ═══════════════════════════════════════════════════════════════════════
  // TODO: Replace with actual game scoring locations

  /** Example scoring position - Left. */
  public static final Pose2d kBlueScoringLeft = new Pose2d(2.0, 5.5, Rotation2d.fromDegrees(0));

  /** Example scoring position - Center. */
  public static final Pose2d kBlueScoringCenter = new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0));

  /** Example scoring position - Right. */
  public static final Pose2d kBlueScoringRight = new Pose2d(2.0, 2.5, Rotation2d.fromDegrees(0));

  /** Example secondary scoring location. */
  public static final Pose2d kBlueSecondaryScoring = new Pose2d(1.5, 7.0, Rotation2d.fromDegrees(-90));

  // ═══════════════════════════════════════════════════════════════════════
  // RED ALLIANCE SCORING POSITIONS (mirrored from blue)
  // ═══════════════════════════════════════════════════════════════════════
  // TODO: Update to match blue alliance positions (mirrored across field centerline)

  /** Example scoring position - Left (mirrored from blue). */
  public static final Pose2d kRedScoringLeft = new Pose2d(14.5, 5.5, Rotation2d.fromDegrees(180));

  /** Example scoring position - Center (mirrored from blue). */
  public static final Pose2d kRedScoringCenter = new Pose2d(14.5, 4.0, Rotation2d.fromDegrees(180));

  /** Example scoring position - Right (mirrored from blue). */
  public static final Pose2d kRedScoringRight = new Pose2d(14.5, 2.5, Rotation2d.fromDegrees(180));

  /** Example secondary scoring location (mirrored from blue). */
  public static final Pose2d kRedSecondaryScoring = new Pose2d(15.0, 7.0, Rotation2d.fromDegrees(90));

  // ═══════════════════════════════════════════════════════════════════════
  // SCORING DISTANCE OFFSETS
  // ═══════════════════════════════════════════════════════════════════════
  // TODO: Update based on robot dimensions and game requirements

  /**
   * Distance to stop from primary scoring target (meters).
   *
   * <p>Robot positions this distance away before deploying scoring mechanism.
   * Adjust based on robot bumper dimensions and game piece release point.
   */
  public static final double kPrimaryScoringDistance = 0.5;

  /**
   * Distance to stop from secondary scoring target (meters).
   *
   * <p>May be different than primary if mechanism geometry differs.
   */
  public static final double kSecondaryScoringDistance = 0.4;

  private FieldConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
