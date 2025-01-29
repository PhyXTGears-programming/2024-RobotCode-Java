package frc.robot.auto;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.auto.load.SubsystemRegistry;
import frc.robot.math.Vector;
import frc.robot.subsystems.drive.Drivetrain;

public class Auto {
    public static ArrayList<Pose2d> loadPosePathFromJSON(/* FIXME: add json loader */ Object path) {
        return new ArrayList();
    }

    public static ArrayList<PathPoint> loadPathFromJSON(/* FIXME: add json loader */ Object path, SubsystemRegistry registry) {
        return new ArrayList();
    }

    public static Command loadPoseFollowCommandFromFile(Drivetrain drivetrain, String filename) {
        // FIXME
        return new RunCommand(() -> {
        });
    }

    public static Command loadPathFollowCommandFromFile(String filename, SubsystemRegistry registry) {
        // FIXME
        return new RunCommand(() -> {
        });
    }

    public static Command generatePathFollowCommand(
        ArrayList<Pose2d> pose,
        LinearVelocity speed,
        Drivetrain drivetrain
    ) {
        var currentPoseIndex = new Object() {
            public int index = 0;
        };
        Vector movementDirection = new Vector();

        return new FunctionalCommand(
                () -> {
                    currentPoseIndex.index = 0;
                    movementDirection.zero();
                },
                () -> {
                    // FIXME: execute
                },
                (isInterrupted) -> {
                    // FIXME: end
                },
                () -> {
                    // FIXME: finished
                    return true;
                },
                drivetrain);
    }

    public static Command generatePathFollowCommand(
            ArrayList<PathPoint> path,
            Drivetrain drivetrain) {
        var currentPoseIndex = new Object() {
            public int index = 0;
        };
        Vector movementDirection = new Vector();
        var haltPointIndex = new Object() {
            public int index = 0;
        };

        return new FunctionalCommand(
                () -> {
                    currentPoseIndex.index = 0;
                    movementDirection.zero();
                    haltPointIndex.index = 0;
                },
                () -> {
                    // FIXME: execute
                },
                (isInterrupted) -> {
                    // FIXME: end
                },
                () -> {
                    // FIXME: finished
                    return true;
                },
                drivetrain);
    }

    public static Command moveBackwardsCommand(Drivetrain drivetrain) {
        ArrayList<Pose2d> path = new ArrayList<Pose2d>();

        path.add(new Pose2d(Units.Meters.of(-0.0), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(-0.1), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(-0.2), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(-0.3), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(-0.4), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(-0.5), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(-0.6), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(-0.7), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(-0.8), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(-0.9), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(-1.0), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(-1.1), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(-1.2), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(-1.3), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(-1.4), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(-1.5), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));

        return generatePathFollowCommand(path, Units.MetersPerSecond.of(1.0), drivetrain);
    }

    public static Command moveForwardsCommand(Drivetrain drivetrain) {
        ArrayList<Pose2d> path = new ArrayList<Pose2d>();
        path.add(new Pose2d(Units.Meters.of(0.0), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(0.1), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(0.2), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(0.3), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(0.4), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(0.5), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(0.6), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(0.7), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(0.8), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(0.9), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(1.0), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(1.1), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(1.2), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(1.3), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(1.4), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));
        path.add(new Pose2d(Units.Meters.of(1.5), Units.Meters.of(0.0), Rotation2d.fromRadians(0.0)));

        return generatePathFollowCommand(path, Units.MetersPerSecond.of(1.0), drivetrain);

    }

}
