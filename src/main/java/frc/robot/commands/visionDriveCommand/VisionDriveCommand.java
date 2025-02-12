package frc.robot.commands.visionDriveCommand;

import static edu.wpi.first.math.MathUtil.clamp;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import  edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;

public class VisionDriveCommand extends Command {
    
    private final int kTargetDriveAprilTagId;
    private final Vision kVisionSubsystem;
    private final Drivetrain kDrivetrain;
    private final Time kRobotPeriod;

    // config

    private final LinearVelocity kStrafeSpeed = Units.MetersPerSecond.of(0.8);
    private final LinearVelocity kForwardSpeed = Units.MetersPerSecond.of(0.8);
    
    private final LinearVelocity kMaxLinearSpeed = Units.MetersPerSecond.of(1.2);

    private final AngularVelocity kMaxAngularSpeed = Units.DegreesPerSecond.of(1.2);
    
    private final double kDistanceMultiplier = 0.04;

    private boolean mIsDoneTurning = false;
    private boolean mIsDoneMoving = false;

    private AngularVelocity angularVelocity = Units.DegreesPerSecond.zero();
    private LinearVelocity stafeVelocity = Units.MetersPerSecond.zero();
    private LinearVelocity forwardVelocity = Units.MetersPerSecond.zero();

    
    public VisionDriveCommand(Vision visionSubsystem, Drivetrain drivetrain, int TargetDriveAprilTagId, Time period){
        kTargetDriveAprilTagId = TargetDriveAprilTagId;
        kVisionSubsystem = visionSubsystem;
        kDrivetrain = drivetrain;
        kRobotPeriod = period;
        // set the filter
        kVisionSubsystem.setFilter(kTargetDriveAprilTagId);
    }

    @Override
    public void initialize() {
        mIsDoneMoving = false;
        mIsDoneTurning = false;
    }

    @Override
    public void execute() {

        Vision.VisionTagData data = kVisionSubsystem.robotDriveToAprilTag(kTargetDriveAprilTagId);

        // see if we can see THE april tag
        if (kVisionSubsystem.isAprilTagDetected()) {

            System.out.println("[drive progress] " + Boolean.toString(mIsDoneMoving) + ", " + Boolean.toString(mIsDoneTurning));
            System.out.printf("[drive progress] fwd dist: %f, strf dist: %f\n", data.mDistanceToAprilTag, data.mTurningDistance);

            // if we are not in the turning thershold
            if (Math.abs(data.mTurningDistance) > Vision.TURNING_THERSHOLD) {
                if (data.mTurningDistance > 0.0) {
                    System.out.println("Turn right");

                    stafeVelocity = (kStrafeSpeed.times(data.mTurningDistance * kDistanceMultiplier));

                } else {
                    System.out.println("Turn left");

                    stafeVelocity = (kStrafeSpeed.times(data.mTurningDistance * kDistanceMultiplier)).negate();
                }

            }
            // this means that we are in the TURNING_THERSHOLD so we can stop turning
            else {
                mIsDoneTurning = true;
            }

            // see if we are in the STOP_MOVING_THERSHOLD if so then stop
            if (Math.abs(data.mDistanceToAprilTag) > Vision.STOP_MOVING_THERSHOLD) {
                // detect if we are too far from the april tag if so then have forwards
                if (data.mDistanceToAprilTag > Vision.DISTANCE_WANTED_FOR_ROBOT) {
                    System.out.println("backwards");

                    // FIXME: use mDistanceToAprilTag as multiplication factor?
                    forwardVelocity = (kForwardSpeed.times(data.mTurningDistance * kDistanceMultiplier)).negate();

                }
                else {
                    System.out.println("forwards");

                    forwardVelocity = (kForwardSpeed.times(data.mTurningDistance * kDistanceMultiplier)).negate();
                }
            }
            else{
                mIsDoneMoving = true;
            }
        }

        // clamp just in case

        forwardVelocity = clamp(forwardVelocity, kMaxLinearSpeed.negate(), kMaxLinearSpeed);
        stafeVelocity = clamp(stafeVelocity, kMaxLinearSpeed.negate(), kMaxLinearSpeed);
        angularVelocity = clamp(angularVelocity, kMaxAngularSpeed.negate(), kMaxAngularSpeed);

        // now drive

        System.out.println(forwardVelocity.in(Units.MetersPerSecond));
        System.out.println(stafeVelocity.in(Units.MetersPerSecond));

        kDrivetrain.Drive(
            forwardVelocity,
            stafeVelocity,
            angularVelocity,
            false,
            kRobotPeriod
        );

    }

    @Override
    public boolean isFinished() {
        return mIsDoneTurning && mIsDoneMoving;
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
}