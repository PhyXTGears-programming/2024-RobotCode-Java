package frc.robot.commands;

import java.util.ArrayList;
import java.util.concurrent.ArrayBlockingQueue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Drive;
import frc.robot.auto.PathPoint;
import frc.robot.auto.PathPoint.PathPointType;
import frc.robot.math.Point;
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

            // Serach for next appearling point.
            // If any new points visited have a halt condition, then look no further.
            for (int i = mCurrentPoseIndex; m < mPath.size(); i ++) {
                // FIXME: set a value to be = mPath.get(i);
            }
        }
        }

    @Override
    public void end(boolean isInterrupted) {

    }

    @Override
    public boolean isFinished() {
        
    }
}
