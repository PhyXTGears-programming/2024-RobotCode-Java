package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.math.Point;
import frc.robot.Constants;
import frc.robot.Interface;
import frc.robot.lib.config.ConfigTable;
import frc.robot.subsystems.drive.SwerveModule.PidConfig;

public class Drivetrain extends SubsystemBase {
    
    private SwerveModule mFrontLeft = null;
    private SwerveModule mFrontRight = null;
    private SwerveModule mBackLeft = null;
    private SwerveModule mBackRight = null;

    private Translation2d mFrontLeftLocation = new Translation2d(Units.Meters.of(+0.287), Units.Meters.of(+0.287));
    private Translation2d mFrontRightLocation = new Translation2d(Units.Meters.of(+0.287), Units.Meters.of(-0.287));
    private Translation2d mBackLeftLocation = new Translation2d(Units.Meters.of(-0.287), Units.Meters.of(+0.287));
    private Translation2d mBackRightLocation = new Translation2d(Units.Meters.of(-0.287), Units.Meters.of(-0.287));

    private SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(
        mFrontLeftLocation, 
        mFrontRightLocation, 
        mBackLeftLocation,
        mBackRightLocation
    );
        
    private AHRS mGyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    private Angle mGyroOffset = Degrees.of(0.0);

    private boolean mIsFieldOriented = true;

    private SwerveDriveOdometry mOdometry;


    public Drivetrain(ConfigTable table) {
        double turnP = 0.0;
        double turnI = 0.0;
        double turnD = 0.0;

        {
            Optional<Double> kP = table.getDouble("swerve.turn.kP");

            if (kP.isEmpty()) {
                System.err.println("Error: drivetrain cannot find toml property swerve.turn.kP");
                throw new Error("error");
            }

            turnP = kP.get();

        }

        {
            Optional<Double> kI = table.getDouble("swerve.turn.kI");

            if (kI.isEmpty()) {
                System.err.println("Error: drivetrain cannot find toml property swerve.turn.kI");
                throw new Error("error");

            }       

            turnI = kI.get();
        }

        {
            Optional<Double> kD = table.getDouble("swerve.turn.kD");

            if (kD.isEmpty()) {
                System.err.println("Error: drivetrain cannot find toml property swerve.turn.kI");
                throw new Error("error");
            }

            turnD = kD.get();
        }

        PidConfig turnPidConfig = new PidConfig(turnP, turnI, turnD);
        
        {           
            Optional<Double> frontLeftAbsEncoderOffset = table.getDouble("frontLeftAbsEncoderOffset");
            
            if (frontLeftAbsEncoderOffset.isEmpty()) {
                System.err.println("Error: drivetrain cannot find toml property frontLeftAbsEncoderOffset");
                throw new Error("error");
            }
            

            mFrontLeft = new SwerveModule(
                Interface.Drive.kFrontLeftDrive,
                Interface.Drive.kFrontLeftTurn,
                Interface.Drive.kFrontLeftEncoder,
                Units.Degrees.of(frontLeftAbsEncoderOffset.get()),
                turnPidConfig,
                "front-left");
        }

        {
            Optional<Double> frontRightAbsEncoderOffset = table.getDouble("frontRightAbsEncoderOffset");

            if (frontRightAbsEncoderOffset.isEmpty()) {

                System.err.println("Error drivetrain cannot find toml property frontRightAbsEncoderOffset");
                throw new Error("error");
            }

            mFrontRight = new SwerveModule(
                Interface.Drive.kFrontRightDrive,
                Interface.Drive.kFrontRightTurn,
                Interface.Drive.kFrontRightEncoder,
                Units.Degrees.of(frontRightAbsEncoderOffset.get()),
                turnPidConfig,
                "front-right"
            );
        }

        {
            Optional<Double> backLeftAbsEncoderOffset = table.getDouble("backLeftAbsEncoderOffset");

            if (backLeftAbsEncoderOffset.isEmpty()) {
                System.err.println("Error: drivetrain cannot find toml property backLeftAbsEncoderOffset");
                throw new Error("error");
            }

            mBackLeft = new SwerveModule(
                Interface.Drive.kBackLeftDrive,
                Interface.Drive.kBackLeftTurn,
                Interface.Drive.kBackLeftEncoder,
                Units.Degrees.of(backLeftAbsEncoderOffset.get()),
                turnPidConfig,
                "back-left"
            );
        }

        {
        
            Optional<Double> backRightAbsEncoderOffset = table.getDouble("backRightAbsEncoderOffset");

            if (backRightAbsEncoderOffset.isEmpty()) {
                System.err.println("Error: drivetrain cannot find toml property backRightAbsEncoderOffset");
                throw new Error("error");
            }

            mBackRight = new SwerveModule(
                Interface.Drive.kBackRightDrive,
                Interface.Drive.kBackRightTurn,
                Interface.Drive.kBackRightEncoder,
                Units.Degrees.of(backRightAbsEncoderOffset.get()),
                turnPidConfig,
                "back-right");
        }

        mGyro.reset();

        Timer deadline = new Timer();
        deadline.start();

        while (mGyro.isCalibrating()) {
            if (deadline.advanceIfElapsed(30)) {
                System.err.println("Error: Drivetrain navx calibration deadline elapsed");
                break;
            }
        }

        resetGyro();

        mOdometry = new SwerveDriveOdometry(
            mKinematics,
            new Rotation2d(getHeading()),
            new SwerveModulePosition[]{
                mFrontLeft.GetPosition(), 
                mFrontRight.GetPosition(),
                mBackLeft.GetPosition(),
                mBackRight.GetPosition()
            }
        );
    } //this bracket ends config table

    public void Drive(
        LinearVelocity forwardSpeed,
        LinearVelocity strafeSpeed,
        AngularVelocity turnSpeed,
        boolean fieldOriented
    ) {
        Drive(
            forwardSpeed,
            strafeSpeed,
            turnSpeed,
            fieldOriented,
            Units.Seconds.of(0.02)
        );
    }

    public void Drive(
        LinearVelocity forwardSpeed, 
        LinearVelocity strafeSpeed, 
        AngularVelocity turnSpeed, 
        boolean isFieldOriented,
        Time period
    ) {

        ChassisSpeeds chassisSpeeds = 
            isFieldOriented
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                forwardSpeed,
                strafeSpeed,
                turnSpeed,
                new Rotation2d(getHeading())
            )
            : new ChassisSpeeds(forwardSpeed, strafeSpeed, turnSpeed);
        
        SwerveModuleState[] states = mKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(chassisSpeeds, period.magnitude())
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drive.kMaxDriveSpeedMetersPerSecond);

        SwerveModuleState fl = states[0];
        SwerveModuleState fr = states[1];
        SwerveModuleState bl = states[2];
        SwerveModuleState br = states[3];

        mFrontLeft.SetDesiredState(fl);
        mFrontRight.SetDesiredState(fr);
        mBackLeft.SetDesiredState(bl);
        mBackRight.SetDesiredState(br);

    }  

    @Override
    public void periodic() {

        SmartDashboard.putBoolean(
            "Is Field Oriented?",
            IsFieldOriented()
        );

        SmartDashboard.putNumber(
            "Front Left Distance (m)",
            mFrontLeft.GetPosition().distanceMeters
        );

        SmartDashboard.putNumber(
            "Robot Heading",
            getHeading().in(Degrees) 
        );

    }

    public boolean IsFieldOriented() {
        return mIsFieldOriented;
    }

    public void setFieldOriented(boolean isFieldOriented) {
        mIsFieldOriented = isFieldOriented;
    }

    public void toggleFieldOriented() {
        mIsFieldOriented = !mIsFieldOriented;
    }

    public void resetGyro() {
        mGyroOffset = Degrees.of(mGyro.getYaw()).unaryMinus();
    }

    public void resetGyroToHeading(Angle heading) {
        mGyroOffset = heading.minus(getHeading());
    }

    public Angle getHeading() {
        return Degrees.of(mGyro.getYaw()).plus(mGyroOffset).unaryMinus(); 
    }

    public void updateOdometry() {

        mOdometry.update(
            new Rotation2d(getHeading()),
            new SwerveModulePosition[]{
            mFrontLeft.GetPosition(),
            mFrontRight.GetPosition(),
            mBackLeft.GetPosition(),
            mBackRight.GetPosition(),
            }
        );
    }

    public void resetPosition() {
        mOdometry.resetPosition(
            new Rotation2d(getHeading()),
            new SwerveModulePosition[] {
                mFrontLeft.GetPosition(),
                mFrontRight.GetPosition(),
                mBackLeft.GetPosition(),
                mBackRight.GetPosition(),
            },
            new Pose2d(0, 0, new Rotation2d(0))
        );
    }

    public void setPosition(Angle heading, Pose2d toPose) {
        mOdometry.resetPosition(
            new Rotation2d(heading), 
            new SwerveModulePosition[]{
                mFrontLeft.GetPosition(),
                mFrontRight.GetPosition(),
                mBackLeft.GetPosition(),
                mBackRight.GetPosition(),
            },
            toPose
        );

    }

    public void setPose(Pose2d toPose) {
        setPosition(getHeading(), toPose);
    }

    public Point getChassisPosition() {
        Translation2d translation = mOdometry.getPoseMeters().getTranslation();
        return new Point(
            translation.getMeasureX().in(Units.Meters), 
            translation.getMeasureY().in(Units.Meters)
        );
    }

    public static SwerveModulePosition getModulePosition(Translation2d pos) {

            double xM = pos.getMeasureX().in(Units.Meters);
            double yM = pos.getMeasureY().in(Units.Meters);

            double distance = Math.sqrt(xM * xM + yM * yM);

            Rotation2d angle = new Rotation2d(xM, yM);

            return new SwerveModulePosition(distance, angle);
    }
}
