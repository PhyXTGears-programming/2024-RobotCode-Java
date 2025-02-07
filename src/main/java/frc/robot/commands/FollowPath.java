package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.math.Point;
import frc.robot.math.Vector;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.ArrayList;
import java.util.concurrent.ArrayBlockingQueue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.PathPoint;
import frc.robot.auto.PathPoint.PathPointType;
import frc.robot.subsystems.drive.Drivetrain;

public class FollowPath extends Command {
    
    private Drivetrain mDrivetrain;
    
    private ArrayList<PathPoint> mPath;
    private int mCurrentPoseIndex;
    private int mHaltPoseIndex;
    private int mLastNearestPoseIndex;
    
    private ArrayBlockingQueue<Command> mCmdQueue = new ArrayBlockingQueue<Command>(32);

    boolean mIsAtHaltPose;

    LinearVelocity mMaxSpeed = Units.MetersPerSecond.of(4);

    public FollowPath(ArrayList<PathPoint> path, Drivetrain drivetrain, LinearVelocity maxSpeed) {

    }
    
    @Override
    public void initialize() {
        mCurrentPoseIndex = 0;
        mHaltPoseIndex = -1;
        mLastNearestPoseIndex = 0;

        // Make sure the queue is empty.
        while (!mCmdQueue.isEmpty()) {
            mCmdQueue.clear();
        }

        mIsAtHaltPose = (mPath.get(0).getType() == PathPointType.Halt);

        if (mPath.get(0).hasCommand()) {
            mCmdQueue.add(mPath.get(0).getCommand().orElseGet(() -> {
                System.err.println("Error: Unable to find command at path index 0");
                return Commands.none();
            }));
        }

        if (mIsAtHaltPose && !mCmdQueue.isEmpty()) {
            mHaltPoseIndex = 0;
        }
        
        mDrivetrain.resetGyroToHeading(mPath.get(0).getRotation().getMeasure());
        
        mDrivetrain.setPosition(
            mPath.get(0).getRotation().getMeasure(), 
            mPath.get(0).getPose()
        );

        if (!mCmdQueue.isEmpty()) {
            System.out.println("Auto: FollowPath: Queue command: Pose 0");
            System.out.println("Auto: FollowPath: Start command");
        }
    }

    @Override
    public void execute() {
        Point currentPoint = mDrivetrain.getChassisPosition();

        if (!mCmdQueue.isEmpty()) {
            if (mCmdQueue.peek().isFinished()) {
                System.out.println("Auto: FollowPath: End command");
                mCmdQueue.peek().end(false);
                mCmdQueue.poll();

                if (!mCmdQueue.isEmpty()) {
                    // Start next command that was queued.
                    System.out.println("Auto: FollowPath: Start command");
                    mCmdQueue.peek().initialize();
                } else if (-1 != mHaltPoseIndex) {
                    // Let path command drive away.
                    mHaltPoseIndex = -1;
                    mIsAtHaltPose = false;
                }
            } else {
                mCmdQueue.peek().execute();
            }

        }else if (mIsAtHaltPose) {
            mHaltPoseIndex = -1;
            mIsAtHaltPose = false;
        }
        if (-1 == mHaltPoseIndex) {
            // No halt point selected.

            // Serach for next appealing point.
            // If any new points visited have a halt condition, then look no further.
            for (int i = mCurrentPoseIndex; i < mPath.size(); i ++) {
                PathPoint pose = mPath.get(i);

                if (pose.getType() == PathPointType.Halt && i != mCurrentPoseIndex) {
                    // Found halt point that is not the current point.
                    mHaltPoseIndex = i;
                    mCurrentPoseIndex = i;
                    mIsAtHaltPose = false;

                    // Break early so we don't look past this stop waypoint.
                    break;
                }
            
            double distance = Math.sqrt(
                Math.pow(pose.getX().in(Meters) - currentPoint.x, 2)
                + Math.pow(pose.getY().in(Meters) -currentPoint.y, 2)
            );

            if (distance > Constants.Auto.kMaxPathPoseDistance.in(Meters)) {
                // Found point just outside max distance.
                mCurrentPoseIndex = i;
                break;
            }

            if (i == (mPath.size() -1)) {
                // No appealing point found before the end of the list.
                // Use the last point.
                mCurrentPoseIndex = i; 
            }


            
            }
        } else {
            // Halt point already selected. Maintain approach to halt point.
            mCurrentPoseIndex =  mHaltPoseIndex; 
        }
        
        int nextNearestPoseIndex = mLastNearestPoseIndex;

        // If any new nearby points have commands, add those commands to the queue.

        // Collect commands up to current nearest index.
        for (int i = mLastNearestPoseIndex; i <= mCurrentPoseIndex && i < mPath.size(); i ++) {
            PathPoint pose = mPath.get(i);
            double distance = Math.sqrt(
                Math.pow(pose.getX().in(Meters) - currentPoint.x, 2)
                + Math.pow(pose.getY().in(Meters) - currentPoint.y, 2)
            );

            if (distance <= Constants.Auto.kNearbyDistanceThreshold.in(Meters)) {
                nextNearestPoseIndex = i;
            }
        }

        // If new nearby point found, collect all commands since last nearest point.
        for (int i = mLastNearestPoseIndex + 1; i <= nextNearestPoseIndex; i ++) {
            PathPoint pose = mPath.get(i);

            if (pose.hasCommand()) {
                System.out.println("Auto: FollowPath: Queue command: Pose" + i);

                if (mCmdQueue.isEmpty()) {
                    // No commands active. Run command immediately.
                    mCmdQueue.add(pose.getCommand().orElseGet((() -> {
                        System.err.println("Error: Unable to find command at pose");
                        return Commands.none();
                    })));
                    System.out.println("Auto: FollowPath: Start command");
                    mCmdQueue.peek().initialize();
                } else {
                    // Another command is active. Only add to queue.
                    mCmdQueue.add(pose.getCommand().orElseGet((() -> {
                        System.err.println("Error: Unable to find command at pose");
                        return Commands.none();
                    })));
                }
            }

        }

        mLastNearestPoseIndex = nextNearestPoseIndex;

        // Movement
        PathPoint selectedPose = mPath.get(mCurrentPoseIndex);

        Point targetPoint = new Point(selectedPose.getX().in(Meters), selectedPose.getY().in(Meters));

        Vector vectorToTarget = (targetPoint.minus(currentPoint));
        Vector directionToTarget = vectorToTarget.unit();
        double distanceToTarget = vectorToTarget.len();

        Vector movementDirection = directionToTarget;

        LinearVelocity speed = selectedPose.getVelocity();
        if (speed.in(MetersPerSecond) > Constants.Drive.kMaxDriveSpeedMetersPerSecond) speed = mMaxSpeed;

        if (-1 != mHaltPoseIndex) {
            if (distanceToTarget < Constants.Auto.kHaltDistanceThreshold.in(Meters)) {
                // Run point commands.
                mIsAtHaltPose = true;
            } else {
                speed = Constants.Auto.kMinSpeed;
            }
        }

        // Rotation
        double currentHeading = mDrivetrain.getHeading().in(Radians);

        double targetRadians = selectedPose.getRotation().getRadians();
        double headingDelta = Math.IEEEremainder((targetRadians - currentHeading), Math.PI * 2.0);

        double rotationSpeed = 0.0;

        if (Math.abs(headingDelta) > Constants.Auto.kRotationDeadZone.getRadians()) {
            rotationSpeed = Math.copySign(clamp(Math.abs(headingDelta) / Math.PI, 0.05, 1), headingDelta
            * Constants.Auto.kRotationSpeed.in(RadiansPerSecond));
        }

        AngularVelocity rotationSpeedRadians = RadiansPerSecond.of(rotationSpeed);

        mDrivetrain.Drive(
            speed.times(movementDirection.x),
            speed.times(movementDirection.y),
            rotationSpeedRadians,
            true,
            Milliseconds.of(20)
        );

    }

    @Override
    public void end(boolean isInterrupted) {
        mDrivetrain.Drive(
            Units.MetersPerSecond.of(0),
            Units.MetersPerSecond.of(0),
            Units.RadiansPerSecond.of(0),
            true,
            Milliseconds.of(20)
        );
    }

    @Override
    public boolean isFinished() {
        if (mCurrentPoseIndex != mPath.size() - 1 || !mCmdQueue.isEmpty()) {
            return false;
        }
        Point currentPoint = mDrivetrain.getChassisPosition();
        final PathPoint selectedPose = mPath.get(mCurrentPoseIndex);
        Point targetPoint = new Point(selectedPose.getX().in(Meters), selectedPose.getY().in(Meters));
        double distanceToTarget = (targetPoint.minus(currentPoint)).len();

        return (distanceToTarget < 0.05);

    
    }
        
private double clamp(double value, double minValue, double maxValue) {
    return (Math.min(Math.max(minValue, value), maxValue));
}

}
