package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;


public final class Constants {

    public final static LinearVelocity kMaxDriveSpeed = Units.MetersPerSecond.of(4.0);

    public final static LinearVelocity kNormalDriveSpeed = Units.MetersPerSecond.of(1.5);
    public final static LinearVelocity kSlowDriveSpeed = kNormalDriveSpeed.times(1.0 - 0.75); // 75% slower
    public final static LinearVelocity kFastDriveSpeed = kMaxDriveSpeed;

    public final static AngularVelocity kMaxTurnSpeed = Units.RadiansPerSecond.of(Math.PI * 2.0);

    public final static AngularVelocity kNormalTurnSpeed = Units.RadiansPerSecond.of(Math.PI);
    public final static AngularVelocity kSlowTurnSpeed = kNormalTurnSpeed.times(1.0 - 0.5); //50% slower
    public final static AngularVelocity kFastTurnSpeed = kMaxTurnSpeed;

    public final static class Drive {
        public final static int kNumberOfSwerverModules = 4;
        public final static Distance kWheelDiamter = Units.Inches.of(3.75);
        public final static double kTurnWheelPerMotorRatio = 21.4285571;
        public final static double kDriveWheelPerMotorRatio = 0.164062;
        public final static double kDriveRpmPerVolt = 78.061404; //FIXME: IDK THE TYPE OF THIS???
        public final static double kDriveRadPerSecPerVolt = -1.0; //FIXME: WHAT ARE THESE TYPES???
    }
}
