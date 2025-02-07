package frc.robot;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public static class Auto {
        public static final Distance kMaxPathPoseDistance = Inches.of(6);
        
        public static final Distance kNearbyDistanceThreshold = Inches.of(12);

        public static final Distance kHaltDistanceThreshold = Meters.of(0.05);
        
        public static final LinearVelocity kMinSpeed = MetersPerSecond.of(0.375);

        public static final Rotation2d kRotationDeadZone = Rotation2d.fromRadians(5);

        public static final AngularVelocity kRotationSpeed = RadiansPerSecond.of(Math.PI/2);
    }
    
    public class Drive {

        public static final double kTrunWheelPerMotorRatio = 0.0;
        
        public static final double kMaxDriveSpeedMetersPerSecond = 4.0;

    }


}
