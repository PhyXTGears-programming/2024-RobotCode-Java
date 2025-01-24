package frc.auto;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;

public class PathPoint {
    public enum PathPointType {
        Run,
        Halt,
    }

    private Measure<Distance> mX;
    private Measure<Distance> mY;
    private Rotation2d mRotation;
    private Measure<Velocity<Distance>> mVelocity; 
    private PathPointType mType;

    private Optional<Command> mCommand;

    public PathPoint(Measure<Distance> x, Measure<Distance> y, Rotation2d rotation,Measure<Velocity<Distance>> velocity) {
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

    public Measure<Distance> getX() {
        return mX;
    }

    public Measure<Distance> getY() {
        return mY;
    }

    public Pose2d getPose() {
        return new Pose2d(getX(), getY(), getRotation());
    }

    public Rotation2d getRotation() {
        return mRotation;
    }

    public Measure<Velocity<Distance>> getVelocity() {
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
