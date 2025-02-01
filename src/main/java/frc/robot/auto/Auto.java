package frc.robot.auto;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.auto.PathPoint.PathPointType;
import frc.robot.auto.load.SubsystemRegistry;
import frc.robot.math.Vector;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.commands.FollowPath;

public class Auto {
    public static ArrayList<Pose2d> loadPosePathFromJSON(JSONArray json) {
            ArrayList<Pose2d> path = new ArrayList();
            
            for (int i = 0; i < json.length(); i ++) {
                JSONObject pathPoint = json.getJSONObject(i);
                double x = pathPoint.getDouble("x");
                double y = pathPoint.getDouble("y");
                double rot = pathPoint.getDouble("rot");
    
                Pose2d pose = new Pose2d(Units.Meters.of(x), Units.Meters.of(y), Rotation2d.fromRadians(rot));
    
                path.add(pose);
        }

        return path;
    }

    public static ArrayList<PathPoint> loadPathFromJSON(JSONArray json, SubsystemRegistry registry) {
        ArrayList<PathPoint> path = new ArrayList();

        for(int i = 0; i < json.length(); i ++) {
            JSONObject pathPoint = json.getJSONObject(i);
            
            double x = pathPoint.getDouble("x");
            double y = pathPoint.getDouble("y");
            double rot = pathPoint.getDouble("rot");
            double vel = pathPoint.getDouble("vel");

            PathPointType pointType = PathPointType.Run;
            if ("stop" == pathPoint.getString("type")) {
                pointType = PathPointType.Halt;
            }
            
            PathPoint point = new PathPoint(Units.Meters.of(x), Units.Meters.of(y), Rotation2d.fromRadians(rot), Units.MetersPerSecond.of(vel));
            
            point.setType(pointType);

            if (pathPoint.has("commands")) {
                JSONObject commands = pathPoint.getJSONObject("commands");

                if (pathPoint.has("rootNode")) {
                    JSONObject command = commands.getJSONObject("rootNode");

                    point.setCommand(command);
                }
            }
            path.add(point);
        }
        return path;
    }

    public static Command loadPoseFollowCommandFromFile(Drivetrain drivetrain, String filename) {
        System.out.println(" Robot: Building path for auto from " + filename);

        try{ 
            String file = Files.readString(Paths.get(filename));
            JSONArray json = new JSONArray(file);

            ArrayList<Pose2d> path = loadPosePathFromJSON(json);
            System.out.println("Robot: Auto path loaded from " + filename);
            
            return generatePathFollowCommand(path, Units.MetersPerSecond.of(1.5), drivetrain);
        
        } catch (IOException e ) {
            System.err.println("Error: Robot: Unable to load path json");
            return Commands.none();
        }

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
