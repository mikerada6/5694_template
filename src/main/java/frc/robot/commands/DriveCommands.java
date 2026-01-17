package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * DRIVE COMMANDS FACTORY
 */
public class DriveCommands {
    private DriveCommands() {}

    // =========================================================================
    // 1. STANDARD DRIVING
    // =========================================================================
    public static Command joystickDrive(
            DriveSubsystem drive,
            DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier rotInput) {

        // Rate Limiters to prevent tipping
        SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
        SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
        SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.kRotationSlewRate);

        return Commands.run(() -> {
            double x = MathUtil.applyDeadband(xInput.getAsDouble(), DriveConstants.kJoystickDeadband);
            double y = MathUtil.applyDeadband(yInput.getAsDouble(), DriveConstants.kJoystickDeadband);
            double rot = MathUtil.applyDeadband(rotInput.getAsDouble(), DriveConstants.kJoystickDeadband);

            // 1. Squaring for fine control
            x = Math.copySign(x * x, x);
            y = Math.copySign(y * y, y);
            rot = Math.copySign(rot * rot, rot);

            // 2. Slew Rate Limiting (Smooth acceleration)
            x = xLimiter.calculate(x);
            y = yLimiter.calculate(y);
            rot = rotLimiter.calculate(rot);

            // 3. Driver Perspective Handling (Alliance Flip)
            // Since Subsystem is "Pure Physics", we handle the "Driver POV" here.
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                x = -x;
                y = -y;
            }

            // 4. Apply Speed Multiplier
            double multiplier = drive.getSpeedMultiplier();

            double xSpeed = x * DriveConstants.kMaxSpeedMetersPerSecond * multiplier;
            double ySpeed = y * DriveConstants.kMaxSpeedMetersPerSecond * multiplier;
            double rotSpeed = rot * DriveConstants.kMaxAngularSpeed * multiplier;

            // 5. Send to Subsystem (use current field-relative mode)
            drive.drive(xSpeed, ySpeed, rotSpeed, drive.getFieldRelative());
        }, drive);
    }


    // =========================================================================
    // 3. PATHFINDING (Uses PathPlanner)
    // =========================================================================
    /**
     * Automatically pathfind to a target position on the field.
     *
     * @param drive The drive subsystem
     * @param targetPose Target position and rotation to reach
     * @return Command that pathfinds to the target
     */
    public static Command driveToPose(DriveSubsystem drive, Pose2d targetPose) {
        // Constraints: Max Vel, Max Accel, Max AngVel, Max AngAccel
        PathConstraints constraints = new PathConstraints(
                DriveConstants.kPathfindMaxVelMetersPerSecond,
                DriveConstants.kPathfindMaxAccelMetersPerSecondSquared,
                DriveConstants.kPathfindMaxAngularVelRadiansPerSecond,
                DriveConstants.kPathfindMaxAngularAccelRadiansPerSecondSquared);

        return AutoBuilder.pathfindToPose(targetPose, constraints, DriveConstants.kPathfindGoalEndVelocity);
    }

    /**
     * Position the robot at a specific distance from a target pose using pathfinding.
     *
     * @param drive The drive subsystem
     * @param target The target pose to position near
     * @param distance Distance to maintain from target (meters)
     * @return Command that positions robot at specified distance
     */
    public static Command positionAtDistance(DriveSubsystem drive, Pose2d target, double distance) {
        // Calculate a pose that is 'distance' meters away from target
        // We maintain the target's rotation (face the same direction)
        Rotation2d angleToTarget = new Rotation2d(
            target.getX() - drive.getCurrentPose().getX(),
            target.getY() - drive.getCurrentPose().getY()
        );

        // Position is 'distance' meters away in the opposite direction
        double offsetX = -distance * angleToTarget.getCos();
        double offsetY = -distance * angleToTarget.getSin();

        Pose2d approachPose = new Pose2d(
            target.getX() + offsetX,
            target.getY() + offsetY,
            target.getRotation()
        );

        return driveToPose(drive, approachPose);
    }

    /**
     * Rotate the robot to face a target pose (no translation).
     *
     * @param drive The drive subsystem
     * @param target The target pose to face
     * @return Command that rotates robot to face target
     */
    public static Command autoAimAtTarget(DriveSubsystem drive, Pose2d target) {
        PIDController rotController = new PIDController(
                DriveConstants.kAutoAimP,
                DriveConstants.kAutoAimI,
                DriveConstants.kAutoAimD);
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        rotController.setTolerance(Math.toRadians(DriveConstants.kAutoAimToleranceDegrees));

        return Commands.run(() -> {
            Pose2d currentPose = drive.getCurrentPose();
            double angleToTarget = Math.atan2(
                target.getY() - currentPose.getY(),
                target.getX() - currentPose.getX()
            );

            double rotSpeed = rotController.calculate(
                currentPose.getRotation().getRadians(),
                angleToTarget
            );

            drive.drive(0, 0, rotSpeed, true);
        }, drive)
        .until(() -> rotController.atSetpoint())
        .withName("AutoAimAt" + String.format("(%.1f,%.1f)", target.getX(), target.getY()));
    }

    /**
     * Drive with joystick translation while automatically aiming at a target.
     * Driver controls movement, robot handles rotation.
     *
     * <p><b>Game-Agnostic</b> - Takes aimOffset as parameter for year-to-year reusability.
     *
     * @param drive The drive subsystem
     * @param xInput Forward/backward speed supplier
     * @param yInput Left/right speed supplier
     * @param target Target pose to lock heading toward
     * @param aimOffsetDegrees Which side of robot to aim (0° = front, 90° = left, 180° = back, -90° = right)
     * @return Command that drives with heading lock
     */
    public static Command driveWithHeadingLock(
            DriveSubsystem drive,
            DoubleSupplier xInput,
            DoubleSupplier yInput,
            Pose2d target,
            double aimOffsetDegrees) {

        PIDController rotController = new PIDController(
                DriveConstants.kHeadingLockP,
                DriveConstants.kHeadingLockI,
                DriveConstants.kHeadingLockD);
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
        SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);

        return Commands.run(() -> {
            // Translation from driver
            double x = MathUtil.applyDeadband(xInput.getAsDouble(), DriveConstants.kJoystickDeadband);
            double y = MathUtil.applyDeadband(yInput.getAsDouble(), DriveConstants.kJoystickDeadband);

            x = xLimiter.calculate(Math.copySign(x * x, x));
            y = yLimiter.calculate(Math.copySign(y * y, y));

            // Alliance flip
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                x = -x;
                y = -y;
            }

            // Apply speed multiplier
            double multiplier = drive.getSpeedMultiplier();
            double xSpeed = x * DriveConstants.kMaxSpeedMetersPerSecond * multiplier;
            double ySpeed = y * DriveConstants.kMaxSpeedMetersPerSecond * multiplier;

            // Auto-rotation toward target
            Pose2d currentPose = drive.getCurrentPose();
            double angleToTarget = Math.atan2(
                target.getY() - currentPose.getY(),
                target.getX() - currentPose.getX()
            );

            // Account for scoring mechanism offset
            double targetAngle = angleToTarget - Math.toRadians(aimOffsetDegrees);

            double rotSpeed = rotController.calculate(
                currentPose.getRotation().getRadians(),
                targetAngle
            );

            // Calculate error manually (getPositionError() is deprecated)
            double error = targetAngle - currentPose.getRotation().getRadians();
            SmartDashboard.putNumber("HeadingLock/ErrorDeg",
                Math.toDegrees(error));

            drive.drive(xSpeed, ySpeed, rotSpeed, true);
        }, drive);
    }

    /**
     * TUNABLE version of driveWithHeadingLock for PID tuning at start of season.
     * PID constants are read from SmartDashboard every loop for real-time tuning.
     *
     * <p><b>Usage:</b> Run this command, adjust PID on dashboard until smooth, then copy
     * final values into Constants for the regular driveWithHeadingLock command.
     *
     * <p><b>Game-Agnostic</b> - Takes aimOffset as parameter for year-to-year reusability.
     *
     * @param drive The drive subsystem
     * @param xInput Forward/backward speed supplier
     * @param yInput Left/right speed supplier
     * @param target Target pose to lock heading toward
     * @param aimOffsetDegrees Which side of robot to aim (0° = front, 90° = left, 180° = back, -90° = right)
     * @return Command with tunable PID for testing
     */
    public static Command driveWithHeadingLockTunable(
            DriveSubsystem drive,
            DoubleSupplier xInput,
            DoubleSupplier yInput,
            Pose2d target,
            double aimOffsetDegrees) {

        // Set default PID values on dashboard from Constants
        SmartDashboard.setDefaultNumber("HeadingLock/kP", DriveConstants.kHeadingLockP);
        SmartDashboard.setDefaultNumber("HeadingLock/kI", DriveConstants.kHeadingLockI);
        SmartDashboard.setDefaultNumber("HeadingLock/kD", DriveConstants.kHeadingLockD);

        PIDController rotController = new PIDController(
                DriveConstants.kHeadingLockP,
                DriveConstants.kHeadingLockI,
                DriveConstants.kHeadingLockD);
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
        SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);

        return Commands.run(() -> {
            // Update PID from dashboard (real-time tuning)
            double kP = SmartDashboard.getNumber("HeadingLock/kP", DriveConstants.kHeadingLockP);
            double kI = SmartDashboard.getNumber("HeadingLock/kI", DriveConstants.kHeadingLockI);
            double kD = SmartDashboard.getNumber("HeadingLock/kD", DriveConstants.kHeadingLockD);
            rotController.setPID(kP, kI, kD);

            // Translation from driver
            double x = MathUtil.applyDeadband(xInput.getAsDouble(), DriveConstants.kJoystickDeadband);
            double y = MathUtil.applyDeadband(yInput.getAsDouble(), DriveConstants.kJoystickDeadband);

            x = xLimiter.calculate(Math.copySign(x * x, x));
            y = yLimiter.calculate(Math.copySign(y * y, y));

            // Alliance flip
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                x = -x;
                y = -y;
            }

            // Apply speed multiplier
            double multiplier = drive.getSpeedMultiplier();
            double xSpeed = x * DriveConstants.kMaxSpeedMetersPerSecond * multiplier;
            double ySpeed = y * DriveConstants.kMaxSpeedMetersPerSecond * multiplier;

            // Auto-rotation toward target
            Pose2d currentPose = drive.getCurrentPose();
            double angleToTarget = Math.atan2(
                target.getY() - currentPose.getY(),
                target.getX() - currentPose.getX()
            );

            // Account for scoring mechanism offset
            double targetAngle = angleToTarget - Math.toRadians(aimOffsetDegrees);

            double rotSpeed = rotController.calculate(
                currentPose.getRotation().getRadians(),
                targetAngle
            );

            // Calculate error manually (getPositionError() is deprecated)
            double error = targetAngle - currentPose.getRotation().getRadians();
            SmartDashboard.putNumber("HeadingLock/ErrorDeg",
                Math.toDegrees(error));
            SmartDashboard.putNumber("HeadingLock/RotSpeed", rotSpeed);

            drive.drive(xSpeed, ySpeed, rotSpeed, true);
        }, drive);
    }

    // =========================================================================
    // 4. SPEED CONTROL
    // =========================================================================
    /**
     * Set speed multiplier to a specific value.
     *
     * @param drive The drive subsystem
     * @param multiplier Speed from 0.0 (stopped) to 1.0 (full speed)
     * @return Command that sets the speed multiplier
     */
    public static Command setSpeed(DriveSubsystem drive, double multiplier) {
        return Commands.runOnce(() -> drive.setSpeedMultiplier(multiplier), drive)
            .withName("SetSpeed_" + (int)(multiplier * 100) + "%");
    }

    /**
     * Set speed to full (100%).
     *
     * @param drive The drive subsystem
     * @return Command that sets full speed
     */
    public static Command fullSpeed(DriveSubsystem drive) {
        return setSpeed(drive, DriveConstants.kFullSpeedMultiplier);
    }

    /**
     * Set speed to half (50%).
     *
     * @param drive The drive subsystem
     * @return Command that sets half speed
     */
    public static Command halfSpeed(DriveSubsystem drive) {
        return setSpeed(drive, DriveConstants.kHalfSpeedMultiplier);
    }

    /**
     * Set speed to quarter (25%).
     *
     * @param drive The drive subsystem
     * @return Command that sets quarter speed
     */
    public static Command quarterSpeed(DriveSubsystem drive) {
        return setSpeed(drive, DriveConstants.kQuarterSpeedMultiplier);
    }

    /**
     * Toggle speed between full (100%) and half (50%).
     *
     * @param drive The drive subsystem
     * @return Command that toggles speed
     */
    public static Command toggleSpeed(DriveSubsystem drive) {
        return Commands.runOnce(() -> {
            double current = drive.getSpeedMultiplier();
            drive.setSpeedMultiplier(current > DriveConstants.kSpeedToggleThreshold
                ? DriveConstants.kHalfSpeedMultiplier
                : DriveConstants.kFullSpeedMultiplier);
        }, drive).withName("ToggleSpeed");
    }

    /**
     * Toggles between field-relative and robot-relative driving.
     *
     * <p><b>Field-relative:</b> Forward is always "away from driver station"
     * <p><b>Robot-relative:</b> Forward is robot's front
     *
     * <p><b>Usage:</b> Bind to driver's left stick trigger
     *
     * @param drive The drive subsystem
     * @return Command that toggles field-relative mode
     */
    public static Command toggleFieldRelative(DriveSubsystem drive) {
        return Commands.runOnce(() -> {
            drive.setFieldRelative(!drive.getFieldRelative());
        }, drive).withName("ToggleFieldRelative");
    }

    // =========================================================================
    // 5. UTILITIES
    // =========================================================================

    public static Command xStance(DriveSubsystem drive) {
        return Commands.run(() -> {
            drive.setModuleStates(new edu.wpi.first.math.kinematics.SwerveModuleState[] {
                    new edu.wpi.first.math.kinematics.SwerveModuleState(0, Rotation2d.fromDegrees(DriveConstants.kXStanceAngleDegrees)),
                    new edu.wpi.first.math.kinematics.SwerveModuleState(0, Rotation2d.fromDegrees(-DriveConstants.kXStanceAngleDegrees)),
                    new edu.wpi.first.math.kinematics.SwerveModuleState(0, Rotation2d.fromDegrees(-DriveConstants.kXStanceAngleDegrees)),
                    new edu.wpi.first.math.kinematics.SwerveModuleState(0, Rotation2d.fromDegrees(DriveConstants.kXStanceAngleDegrees))
            });
        }, drive);
    }

    public static Command zeroHeading(DriveSubsystem drive) {
        return Commands.runOnce(drive::zeroHeading, drive).withName("ZeroHeading");
    }


    // =========================================================================
    // 5. SNAP TO ANGLE (Generic Helper + Public Methods)
    // =========================================================================
    /**
     * Generic snap-to-angle command - DRY helper for cardinal and diamond commands.
     * Snaps robot rotation to nearest multiple of angleIncrementDegrees + offsetDegrees.
     *
     * <p><b>Example usage:</b>
     * <ul>
     *   <li>Cardinal (0, 90, 180, 270): increment=90, offset=0</li>
     *   <li>Diamond (45, 135, 225, 315): increment=90, offset=45</li>
     * </ul>
     *
     * @param drive The drive subsystem
     * @param xInput Forward/backward speed supplier
     * @param yInput Left/right speed supplier
     * @param angleIncrementDegrees Snap increment (typically 90 degrees)
     * @param offsetDegrees Offset from zero (0 for cardinal, 45 for diamond)
     * @return Command that drives with snapped rotation
     */
    private static Command snapToClosestAngle(
            DriveSubsystem drive,
            DoubleSupplier xInput,
            DoubleSupplier yInput,
            double angleIncrementDegrees,
            double offsetDegrees) {

        PIDController turnPID = new PIDController(
                DriveConstants.kRotationPID.kP,
                DriveConstants.kRotationPID.kI,
                DriveConstants.kRotationPID.kD);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);

        SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
        SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);

        return Commands.run(() -> {
            // 1. Standard Translation
            double x = MathUtil.applyDeadband(xInput.getAsDouble(), DriveConstants.kJoystickDeadband);
            double y = MathUtil.applyDeadband(yInput.getAsDouble(), DriveConstants.kJoystickDeadband);
            x = xLimiter.calculate(Math.copySign(x * x, x));
            y = yLimiter.calculate(Math.copySign(y * y, y));

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                x = -x;
                y = -y;
            }

            // 2. Calculate Closest Snap Angle
            Rotation2d currentRot = drive.getRotation2d();
            double currentDeg = currentRot.getDegrees();

            // Snap to nearest multiple: round((angle - offset) / increment) * increment + offset
            double snappedDeg = Math.round((currentDeg - offsetDegrees) / angleIncrementDegrees)
                    * angleIncrementDegrees + offsetDegrees;

            Rotation2d targetRot = Rotation2d.fromDegrees(snappedDeg);

            // 3. PID Control
            double rotSpeed = turnPID.calculate(currentRot.getRadians(), targetRot.getRadians());

            drive.drive(
                    x * DriveConstants.kMaxSpeedMetersPerSecond,
                    y * DriveConstants.kMaxSpeedMetersPerSecond,
                    rotSpeed,
                    true
            );
        }, drive);
    }

    /**
     * Snaps the robot to the closest cardinal direction (0, 90, 180, 270).
     * Useful for aligning with the source or walls.
     *
     * @param drive The drive subsystem
     * @param xInput Forward/backward speed supplier
     * @param yInput Left/right speed supplier
     * @return Command that snaps to cardinal directions
     */
    public static Command snapToClosestCardinal(
            DriveSubsystem drive,
            DoubleSupplier xInput,
            DoubleSupplier yInput) {
        return snapToClosestAngle(drive, xInput, yInput,
            DriveConstants.kCardinalSnapIncrementDegrees,
            DriveConstants.kCardinalSnapOffsetDegrees);
    }

    /**
     * Snaps the robot to the closest diagonal angle (45, 135, 225, 315).
     * This puts the robot in a "Diamond" orientation, which is ideal for
     * climbing the Cage/Ramp so one wheel contacts the incline at a time.
     *
     * @param drive The drive subsystem
     * @param xInput Forward/backward speed supplier
     * @param yInput Left/right speed supplier
     * @return Command that snaps to diamond directions
     */
    public static Command snapToDiamond(
            DriveSubsystem drive,
            DoubleSupplier xInput,
            DoubleSupplier yInput) {
        return snapToClosestAngle(drive, xInput, yInput,
            DriveConstants.kCardinalSnapIncrementDegrees,
            DriveConstants.kDiamondSnapOffsetDegrees);
    }

    public static Command forceVisionReset(DriveSubsystem drive) {
        return Commands.runOnce(() -> {
            boolean success = drive.forceVisionReset();
            // Optional: Rumble controller if success
        }, drive).withName("ForceVisionReset");
    }
}