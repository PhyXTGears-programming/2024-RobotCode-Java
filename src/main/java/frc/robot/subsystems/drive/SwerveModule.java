package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.AngularAcceleration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.Constants;

public class SwerveModule {
    String mName;

    Angle mAbsEncoderOffset = Degrees.zero();

    SparkMax mDriveMotor;
    SparkClosedLoopController mDrivePid;
    RelativeEncoder mDriveEncoder;

    SparkMax mTurnMotor;
    SparkClosedLoopController mTurnPid;
    RelativeEncoder mTurnEncoder;

    CANcoder mTurnAbsEncoder;
    StatusSignal<Angle> mTurnAbsPositionSignal;

    public static final AngularVelocity kModuleMaxAngularVelocity = Units.RotationsPerSecond.of(0.5);

    public static final AngularAcceleration kModuleMaxAnguularAcceleration = Units.RotationsPerSecond
            .per(Units.Second).of(1.0);

    public SwerveModule(
        int driveMotorCan,
        int turnMotorCan,
        int turnAbsEncoderCan,
        Angle absEncoderOffset,
        PidConfig turnConfig,
        String name
    ) {
        mName = name;

        mAbsEncoderOffset = absEncoderOffset;

        mDriveMotor = new SparkMax(driveMotorCan, MotorType.kBrushless);
        mDrivePid = mDriveMotor.getClosedLoopController();
        mDriveEncoder = mDriveMotor.getEncoder();

        SparkMaxConfig driveMotorConfig = new SparkMaxConfig();

        driveMotorConfig.closedLoop
            .pidf (
                1.0, //SetP
                0.0, //SetI
                0.0, //SetD
                0.0 //SetFF
            );

        mDriveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mTurnMotor = new SparkMax(turnMotorCan, MotorType.kBrushless);
        
        SparkMaxConfig turnMotorConfig = new SparkMaxConfig();
        // names are diffrent ill try my best to copy! -Weston Justice
        turnMotorConfig.inverted(true)
            // Set coast mode so we can straighten the wheels at start of match
            // Will switch to brake mode later
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(30);

        turnMotorConfig.encoder
            .positionConversionFactor(
                2.0 * Math.PI
                / Constants.Drive.kTurnWheelPerMotorRatio
            )
            .velocityConversionFactor(
                (2.0 * Math.PI / 1.0)
                / Constants.Drive.kTurnWheelPerMotorRatio
                * (1.0 / 60.0)
            );

        turnMotorConfig.closedLoop
            .outputRange(-1.0, 1.0)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(
                turnConfig.kP, //SetP
                turnConfig.kI, //SetI
                turnConfig.kD, //SetD
                0.0  //SetFF
            )
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-Math.PI, Math.PI);


        mTurnMotor.configure(turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mTurnPid = mTurnMotor.getClosedLoopController();
        mTurnEncoder = mTurnMotor.getEncoder();

        mTurnAbsEncoder = new CANcoder(turnAbsEncoderCan);
        mTurnAbsPositionSignal = mTurnAbsEncoder.getAbsolutePosition();

        CANcoderConfiguration configCanCoder = new CANcoderConfiguration();
        MagnetSensorConfigs configMagnetSensor = new MagnetSensorConfigs();
        configMagnetSensor
            .withAbsoluteSensorDiscontinuityPoint(0.5)
            .withSensorDirection(com.ctre.phoenix6.signals.SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(0.0);

        configCanCoder.withMagnetSensor(configMagnetSensor);
        
        while (true) {
            com.ctre.phoenix6.StatusCode sc = mTurnAbsEncoder.getConfigurator().apply(
                configCanCoder,
                5.0
            );

            if (sc.isOK()) {
                break;
            } else {
                System.err.println(
                    String.format(
                        "!!! ERROR !!! swerve module: failed to apply config to abs encoder for module %s: status code %s: description %s",
                        mName,
                        sc.getName(),
                        sc.getDescription()
                    )
                );
            }

            try {
                Thread.sleep(50);
            } catch (Exception e) {
                // do nothing.
            }
        }

        mDriveEncoder.setPosition(0.0);

        while (true) {
            mTurnAbsPositionSignal.waitForUpdate(5.0);

            com.ctre.phoenix6.StatusCode sc = mTurnAbsPositionSignal.getStatus();
            
            if (sc.isOK()) {
                break;
            } else {
                System.err.println(
                    String.format(
                        "!!! ERROR !!! swerve module: failed to update measurement from abs encoder for module {}: status code {}: description {}",
                        mName,
                        sc.getName(),
                        sc.getDescription()
                    )
                );

            }

            try {
                Thread.sleep(50);
            } catch (Exception e) {
                // Do nothing.
            }
        }

        ResetTurnPosition();

    }

    public void Periodic(){
        mTurnAbsPositionSignal.refresh();
    }
    
    public SwerveModuleState GetState() {
        return new SwerveModuleState(
            mDriveEncoder.getVelocity(),
            Rotation2d.fromRadians(GetTurnPosition().in(Units.Radians))
        );
    }
    
    public SwerveModulePosition GetPosition() {
        return new SwerveModulePosition(
            mDriveEncoder.getPosition(),
            Rotation2d.fromRadians(GetTurnPosition().in(Units.Radians))
        );
    } 

    public void SetDesiredState(
        SwerveModuleState desiredState
    ) {
        Rotation2d encoderRotation = new Rotation2d( GetTurnPosition() );

        // opitmize the reference state to avoid spinning further than 90 deg

        desiredState.optimize(encoderRotation);

        //  scale speed by cosine of angle error this scales down movement
        //  perpendicular to the desired direction of travel that can occur when
        //  modules change directions. this results in smoother driving

        desiredState.speedMetersPerSecond *= (desiredState.angle.minus(encoderRotation)).getCos();

        // Set drive speed percent.

        double drivePercent = clamp(
            (desiredState.speedMetersPerSecond / Constants.kMaxDriveSpeed.in(Units.MetersPerSecond)),
            -1.0,
             1.0
        );

        mDriveMotor.set(drivePercent);

        mTurnPid.setReference(
            desiredState.angle.getRadians(),
            com.revrobotics.spark.SparkBase.ControlType.kPosition
        );

    }

    public void ResetTurnPosition() {
        mTurnEncoder.setPosition(GetTurnAbsPosition().in(Radians));
    }

    public Angle GetTurnPosition() {
        return Units.Radians.of( mTurnEncoder.getPosition() );
    }

    public Angle GetTurnAbsPosition() {
        double absA = GetTurnAbsPositionRaw().plus(mAbsEncoderOffset).plus(Degrees.of(180.0)).in(Degrees);
        double absB = absA % 360.0;
        double absC = absB - 180.0;

        return Degrees.of(absC);
    }

    public Angle GetTurnAbsPositionRaw() {
        Angle position = mTurnAbsPositionSignal.getValue();
        return position;
    }

    public void SetTurnBreak(Boolean isEnabled, SparkMaxConfig turnMotorConfig){
        if(isEnabled){
            turnMotorConfig.idleMode(IdleMode.kBrake);
        }
        else{
            turnMotorConfig.idleMode(IdleMode.kCoast);
        }
    }

    public static final class PidConfig {
        public double kP = 0.0;
        public double kI = 0.0;
        public double kD = 0.0;

        public PidConfig(double p, double i, double d) {
            kP = p;
            kI = i;
            kD = d;
        }
    }

    private double fmod(double a, double b) {
        b = Math.abs(b);
        double quotient = a/b;
        double remainder = quotient - Math.floor(quotient);
        double answer = remainder * b;
        if (answer < 0) {
            answer += b;
        }
        return Math.copySign(answer, a);
    }

    public double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}