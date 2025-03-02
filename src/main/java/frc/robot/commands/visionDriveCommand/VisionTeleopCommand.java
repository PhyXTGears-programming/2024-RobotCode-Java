package frc.robot.commands.visionDriveCommand;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;

public class VisionTeleopCommand extends Command {

    // config
    // for now instead of a toml FIXME:
    public static final double STOP_MOVING_THERSHOLD = 0.58;
    public static final double STAFE_THERSHOLD = 1.5;
    public static final double TURNING_THERSHOLD = 5.0;

    public double mOffset;

    private final Vision kVisionSubsystem;
    private final Drivetrain kDrivetrain;
    private final Time kRobotPeriod;

    // config

    private final LinearVelocity kStrafeSpeed = Units.MetersPerSecond.of(0.22);
    private final AngularVelocity kTurnSpeed = Units.RadiansPerSecond.of(0.58);
    private final LinearVelocity kForwardSpeed = Units.MetersPerSecond.of(1.0);

    private final LinearVelocity kMaxLinearSpeed = Units.MetersPerSecond.of(1.9);

    private final AngularVelocity kMaxAngularSpeed = Units.RadiansPerSecond.of(1.2);

    private final double kDistanceMultiplierForwards = 0.65;
    private final double kDistanceMultiplierStafing = 0.07;
    private final double kDistanceMultiplierTurning = 0.4;

    private final LinearVelocity kMinMoveSpeedLinear = Units.MetersPerSecond.of(0.03);
    private final AngularVelocity kMinMoveSpeedAngular = Units.RadiansPerSecond.of(0.1);

    private final Timer kTimer = new Timer();
    private final double limelightInitFailedTime = 0.4;

    private boolean mIsDoneStafing = false;
    private boolean mIsDoneMoving = false;

    private boolean mDoneInitTeleop = false;

    private int mTargetDriveAprilTagId;

    private AngularVelocity mAngularVelocity = Units.RadiansPerSecond.zero();
    private LinearVelocity mStafeVelocity = Units.MetersPerSecond.zero();
    private LinearVelocity mForwardVelocity = Units.MetersPerSecond.zero();

    public VisionTeleopCommand(
            Vision visionSubsystem,
            Drivetrain drivetrain,
            int targetDriveAprilTagId,
            Time period) {
        mTargetDriveAprilTagId = targetDriveAprilTagId;
        kVisionSubsystem = visionSubsystem;
        kDrivetrain = drivetrain;
        kRobotPeriod = period;

        addRequirements(kDrivetrain, kVisionSubsystem);
    }

    @Override
    public void initialize() {
        // reset some vars
        mIsDoneMoving = false;
        mIsDoneStafing = false;
        mDoneInitTeleop = false;

        // reset the timer
        kTimer.reset();
        kTimer.start();
    }

    @Override
    public void execute() {

        if (!mDoneInitTeleop) {

            if ((int) kVisionSubsystem.mLimelightTable.getEntry("priorityid").getInteger(-1) == -1
                    && kVisionSubsystem.getTagId() != -1) {

                mDoneInitTeleop = true;

                kVisionSubsystem.setFilter(kVisionSubsystem.getTagId());

                mTargetDriveAprilTagId = kVisionSubsystem.getTagId();

                System.out.println("Going to: " + kVisionSubsystem.getTagId());
            }
        }

        if (mTargetDriveAprilTagId == -1 || !mDoneInitTeleop)
            return;

        Vision.VisionTagData data = kVisionSubsystem.robotDriveToAprilTag(mTargetDriveAprilTagId, mOffset);

        double heading = kDrivetrain.getHeading().in(Units.Degree);
        double targetHeading = fmod(Vision.rotations[mTargetDriveAprilTagId - 1], 360.0, 180.0);
        double distanceToTargetHeading = fmod(targetHeading - heading, 360.0, 180.0);

        mAngularVelocity = Units.RadiansPerSecond.zero();
        mStafeVelocity = Units.MetersPerSecond.zero();
        mForwardVelocity = Units.MetersPerSecond.zero();

        // init teleop

        kVisionSubsystem.setFilter(-1);

        // System.out.println("[drive progress] " + Boolean.toString(mIsDoneMoving) + ",
        // " + Boolean.toString(mIsDoneStafing));

        // System.out.println( heading + "-> " + targetHeading + " [DISTANCE] " +
        // distanceToTargetHeading + ", " + Boolean.toString(distanceToTargetHeading >
        // Vision.TURNING_THERSHOLD));

        // turn code
        // detect if we are not done stafing and moving then try and rotate the robot to
        // face the april tag
        if (!(mIsDoneMoving && mIsDoneStafing) && Math.abs(distanceToTargetHeading) > TURNING_THERSHOLD) {
            if (distanceToTargetHeading > 0.0) {
                mAngularVelocity = kTurnSpeed.times(distanceToTargetHeading * kDistanceMultiplierTurning)
                        .plus(kMinMoveSpeedAngular);
            } else {
                mAngularVelocity = kTurnSpeed.times(distanceToTargetHeading * kDistanceMultiplierTurning)
                        .minus(kMinMoveSpeedAngular);
            }
        }

        SmartDashboard.putNumber("target heading", targetHeading);
        SmartDashboard.putNumber("heading error", distanceToTargetHeading);

        // see if we can see THE april tag
        if (kVisionSubsystem.isAprilTagDetected()) {
            // System.out.printf("[drive progress] fwd dist: %f, strf dist: %f\n",
            // data.mDistanceToAprilTag, data.mTurningDistance);
            if (!(mIsDoneStafing && mIsDoneMoving)) {

                SmartDashboard.putNumber("Strafe Distance", data.mTurningDistance);
                SmartDashboard.putNumber("Forward Distance", data.mDistanceToAprilTag);

                // if we are not in the turning thershold
                if (Math.abs(data.mTurningDistance) > STAFE_THERSHOLD) {
                    if (data.mTurningDistance > 0.0) {
                        // System.out.println("Turn right");

                        mStafeVelocity = kStrafeSpeed
                                .times(Math.abs(data.mTurningDistance * kDistanceMultiplierStafing))
                                .plus(kMinMoveSpeedLinear);

                    } else {
                        // System.out.println("Turn left");

                        mStafeVelocity = kStrafeSpeed
                                .times(Math.abs(data.mTurningDistance * kDistanceMultiplierStafing))
                                .plus(kMinMoveSpeedLinear).negate();

                    }

                }
                // this means that we are in the STAFE_THERSHOLD so we can stop stafing
                else {
                    mIsDoneStafing = true;
                }

                // see if we are in the STOP_MOVING_THERSHOLD if so then stop
                if (Math.abs(0.0 - data.mDistanceToAprilTag) > STOP_MOVING_THERSHOLD) {
                    // detect if we are too far from the april tag if so then have forwards
                    if (data.mDistanceToAprilTag > 0.0) {
                        // System.out.println("backwards");

                        mForwardVelocity = kForwardSpeed
                                .times(Math.abs(data.mDistanceToAprilTag * kDistanceMultiplierForwards))
                                .plus(kMinMoveSpeedLinear).negate();

                    } else {
                        // System.out.println("forwards");

                        mForwardVelocity = kForwardSpeed
                                .times(Math.abs(data.mDistanceToAprilTag * kDistanceMultiplierForwards))
                                .plus(kMinMoveSpeedLinear);
                    }
                } else {
                    mIsDoneMoving = true;
                }
            }
        }

        // clamp just in case

        mForwardVelocity = clamp(mForwardVelocity, kMaxLinearSpeed.negate(), kMaxLinearSpeed);
        mStafeVelocity = clamp(mStafeVelocity, kMaxLinearSpeed.negate(), kMaxLinearSpeed);
        mAngularVelocity = clamp(mAngularVelocity, kMaxAngularSpeed.negate(), kMaxAngularSpeed);

        // now drive

        kDrivetrain.Drive(
                mForwardVelocity,
                mStafeVelocity,
                mAngularVelocity,
                false,
                kRobotPeriod);

    }

    @Override
    public boolean isFinished() {
        boolean limelightInitFailed = (kTimer.get() >= limelightInitFailedTime && !mDoneInitTeleop);

        if (mIsDoneMoving && !mIsDoneStafing) {
            System.out.println("stafing is holding us up");
        } else if (mIsDoneStafing && !mIsDoneMoving) {
            System.out.println("moving is holding us up");
        }

        if (limelightInitFailed) {
            System.out.println("Limelight init failed");
            return true;

        } else if ((mIsDoneStafing && mIsDoneMoving)) {

            System.out.println("Done: " + kTimer.get());

            return true;
        }
        return false;
    }

    @Override
    public void end(boolean i) {
        kDrivetrain.Drive(
                Units.MetersPerSecond.zero(),
                Units.MetersPerSecond.zero(),
                Units.RadiansPerSecond.zero(),
                false,
                kRobotPeriod);
    }

    private static LinearVelocity clamp(LinearVelocity _value, LinearVelocity _min, LinearVelocity _max) {
        double value = _value.in(MetersPerSecond);
        double min = _min.in(MetersPerSecond);
        double max = _max.in(MetersPerSecond);

        return MetersPerSecond.of(MathUtil.clamp(value, min, max));
    }

    private static AngularVelocity clamp(AngularVelocity _value, AngularVelocity _min, AngularVelocity _max) {
        double value = _value.in(RadiansPerSecond);
        double min = _min.in(RadiansPerSecond);
        double max = _max.in(RadiansPerSecond);

        return RadiansPerSecond.of(MathUtil.clamp(value, min, max));
    }

    private static double fmod(double a, double b, double bias) {
        double absA = a + bias;
        double absB = absA % b;
        if (absB < 0.0) {
            absB += Math.abs(b);
        }
        double absC = absB - bias;

        return absC;
    }
}