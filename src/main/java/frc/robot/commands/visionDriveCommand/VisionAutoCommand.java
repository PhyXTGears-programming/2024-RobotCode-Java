package frc.robot.commands.visionDriveCommand;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;

public class VisionAutoCommand extends Command {

    // vars

    private double mOffset;

    private final Vision kVisionSubsystem;
    private final Drivetrain kDrivetrain;
    private final Time kRobotPeriod;

    private boolean mIsDoneStafing = false;
    private boolean mIsDoneMoving = false;
    private boolean mDoneAutoInit = false;

    private int mTargetDriveAprilTagId;

    private AngularVelocity mAngularVelocity = Units.RadiansPerSecond.zero();
    private LinearVelocity mStafeVelocity = Units.MetersPerSecond.zero();
    private LinearVelocity mForwardVelocity = Units.MetersPerSecond.zero();

    private Vision.Config mConfig;

    public VisionAutoCommand(
            Vision visionSubsystem,
            Drivetrain drivetrain,
            int targetTagId1,
            int targetTagId2,
            Time period) {

        kVisionSubsystem = visionSubsystem;
        kDrivetrain = drivetrain;
        kRobotPeriod = period;

        mConfig = kVisionSubsystem.getConfigCopy();

        addRequirements(kDrivetrain, kVisionSubsystem);
    }

    public void setOffset(Vision.Alignment alignment) {
        mOffset = mConfig.getOffset(alignment);
    }

    @Override
    public void initialize() {
        // reset some vars
        mIsDoneMoving = false;
        mIsDoneStafing = false;

    }

    @Override
    public void execute() {

        if (mTargetDriveAprilTagId == -1)
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
        if (!(mIsDoneMoving && mIsDoneStafing) && Math.abs(distanceToTargetHeading) > mConfig.TURNING_THERSHOLD) {
            if (distanceToTargetHeading > 0.0) {
                mAngularVelocity = mConfig.kTurnSpeed.times(distanceToTargetHeading * mConfig.kDistanceMultiplierTurning)
                        .plus(mConfig.kMinMoveSpeedAngular);
            } else {
                mAngularVelocity = mConfig.kTurnSpeed.times(distanceToTargetHeading * mConfig.kDistanceMultiplierTurning)
                        .minus(mConfig.kMinMoveSpeedAngular);
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
                if (Math.abs(data.mTurningDistance) > mConfig.STAFE_THERSHOLD) {
                    if (data.mTurningDistance > 0.0) {
                        // System.out.println("Turn right");

                        mStafeVelocity = mConfig.kStrafeSpeed
                                .times(Math.abs(data.mTurningDistance * mConfig.kDistanceMultiplierStafing))
                                .plus(mConfig.kMinMoveSpeedLinear);

                    } else {
                        // System.out.println("Turn left");

                        mStafeVelocity = mConfig.kStrafeSpeed
                                .times(Math.abs(data.mTurningDistance * mConfig.kDistanceMultiplierStafing))
                                .plus(mConfig.kMinMoveSpeedLinear).negate();

                    }

                }
                // this means that we are in the STAFE_THERSHOLD so we can stop stafing
                else {
                    mIsDoneStafing = true;
                }

                // see if we are in the STOP_MOVING_THERSHOLD if so then stop
                if (Math.abs(0.0 - data.mDistanceToAprilTag) > mConfig.STOP_MOVING_THERSHOLD) {
                    // detect if we are too far from the april tag if so then have forwards
                    if (data.mDistanceToAprilTag > 0.0) {
                        // System.out.println("backwards");

                        mForwardVelocity = mConfig.kForwardSpeed
                                .times(Math.abs(data.mDistanceToAprilTag * mConfig.kDistanceMultiplierForwards))
                                .plus(mConfig.kMinMoveSpeedLinear).negate();

                    } else {
                        // System.out.println("forwards");

                        mForwardVelocity = mConfig.kForwardSpeed
                                .times(Math.abs(data.mDistanceToAprilTag * mConfig.kDistanceMultiplierForwards))
                                .plus(mConfig.kMinMoveSpeedLinear);
                    }
                } else {
                    mIsDoneMoving = true;
                }
            }
        }

        // clamp just in case

        mForwardVelocity = clamp(mForwardVelocity, mConfig.kMaxLinearSpeed.negate(), mConfig.kMaxLinearSpeed);
        mStafeVelocity = clamp(mStafeVelocity, mConfig.kMaxLinearSpeed.negate(), mConfig.kMaxLinearSpeed);
        mAngularVelocity = clamp(mAngularVelocity, mConfig.kMaxAngularSpeed.negate(), mConfig.kMaxAngularSpeed);

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
        // if (mIsDoneMoving && !mIsDoneStafing) {
        //     System.out.println("stafing is holding us up");
        // } else if (mIsDoneStafing && !mIsDoneMoving) {
        //     System.out.println("moving is holding us up");
        // }

        if ((mIsDoneStafing && mIsDoneMoving)) {

            System.out.println("Done");

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