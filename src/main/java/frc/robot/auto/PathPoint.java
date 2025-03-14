package frc.robot.auto;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;

public class PathPoint {
    public enum PathPointType {
        Run,
        Halt,
    }

    private Distance mX;
    private Distance mY;
    private Rotation2d mRotation;
    private LinearVelocity mVelocity; 
    private PathPointType mType;

    private Optional<Command> mCommand;

    public PathPoint(Distance x, Distance y, Rotation2d rotation, LinearVelocity velocity) {
    }

    public PathPoint setCommand(Command command) {
        mCommand = Optional.of(command);
        return this;
    }

    public PathPoint setCommand(Optional<Command> command) {
        mCommand = command;
        return this;
    }

    public PathPoint setType(PathPointType type) {
           mType = type;
           return this;
    }

    public Distance getX() {
        return mX;
    }

    public Distance getY() {
        return mY;
    }

    public Pose2d getPose() {
        return new Pose2d(getX(), getY(), getRotation());
    }

    public Rotation2d getRotation() {
        return mRotation;
    }

    public LinearVelocity getVelocity() {
        return mVelocity;
    }

    public PathPointType getType() {
        return mType;
    }

    public Optional<Command> getCommand() {
        return mCommand;
    }

    public boolean hasCommand() {
        return mCommand.isPresent();
    }

    

}
