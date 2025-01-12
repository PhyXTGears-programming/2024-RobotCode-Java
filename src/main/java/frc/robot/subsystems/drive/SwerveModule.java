package frc.robot.subsystems.drive;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Velocity;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class SwerveModule {
    String mName;

    SparkMax mDriveMotor;
    SparkClosedLoopController mDrivePid;
    RelativeEncoder mDriveEncoder;

    SparkMax mTurnMotor;
    SparkClosedLoopController mTurnPid;
    RelativeEncoder mTurnEncoder;

    CANcoder mTurnAbsEncoder;
    StatusSignal<Double> mTurnAbsPositionSignal;

    public static final AngularVelocity kModuleMaxAngularVelocity = Units.RotationsPerSecond.of(0.5);

    public static final AngularAcceleration kModuleMaxAnguularAcceleration = Units.RotationsPerSecond
            .per(Units.Second).of(1.0);

    public SwerveModule(int driveMotorCan,
            int turnMotorCan,
            int turnAbsEncoderCan,
            Angle absEncoderOffset,
            PidConfig turnConfig,
            String name
    ) 
    
    {
        mName = name;

        mDriveMotor = new SparkMax(driveMotorCan, MotorType.kBrushless);
        mDrivePid = mDriveMotor.getClosedLoopController();
        mDriveEncoder = mDriveMotor.getEncoder();

        mTurnMotor = new SparkMax(turnMotorCan, MotorType.kBrushless);
        mTurnPid = mTurnMotor.getClosedLoopController();
        mTurnEncoder = mTurnMotor.getEncoder();

        mTurnAbsEncoder = new CANcoder(turnAbsEncoderCan);
        mTurnAbsPositionSignal = mTurnAbsEncoder.getAbsolutePosition();
        
        SparkMaxConfig turnMotorConfig = new SparkMaxConfig();
        // names are diffrent ill try my best to copy! -Weston Justice
        turnMotorConfig.inverted(true)
            // Set coast mode so we can straighten the wheels at start of match
            // Will switch to brake mode later
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(30);

        //nulls needed for function... to function
        mTurnMotor.configure(turnMotorConfig, null, null);

        CANcoderConfiguration configCanCoder = new CANcoderConfiguration();
        MagnetSensorConfigs configMagnetSensor = new MagnetSensorConfigs();
        configMagnetSensor
            .withAbsoluteSensorDiscontinuityPoint(0.5)
            .withSensorDirection(com.ctre.phoenix6.signals.SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(0.0);

        configCanCoder.withMagnetSensor(configMagnetSensor);
        
        while (true){
            // FIXME: will fix code is diffrent in java, if theres a problem check here
            com.ctre.phoenix6.StatusCode sc = mTurnAbsEncoder.getConfigurator().apply(configMagnetSensor, 5.0);

            if(sc.isOK()){
                break;
            }
            else{
                System.err.println(
                    String.format("!!! ERROR !!! swerve module: failed to apply config to abs encoder for module %s: status code %s: description %s", mName, sc.getName(), sc.getDescription()));
            }
            try{
                Thread.sleep(50);
            } catch(Exception e){
                // do nothing.
            }

        }

        // FIXME: Constants have not been set up yet


        mTurnEncoder.setPositionConversionFactor(
            2.0 * Math.PI
            / Constants.Drive.kTrunWheelPerMotorRatio
        );

        mTurnEncoder.setVelocityConversionFactor(
            (2.0 * Math.PI / 1.0)
            / Constants.Drive.kTrunWheelPerMotorRatio
            * (1.0 / 60.0)
        );

        mTurnPid.setFeedbackDevice(mTurnEncoder);

        // drive pid parameters

        mDrivePid.setP(1.0);
        mDrivePid.setI(0.0);
        mDrivePid.setIZone(0.0);
        mDrivePid.setD(0.0);

        mDrivePid.setFF(0.0);

        mDriveEncoder.setPosition(0.0);

        while (true) {
            mTurnAbsPositionSignal.waitForUpdate(5.0);

            com.ctre.phoenix6.StatusCode sc = mTurnAbsPositionSignal.getStatus();

            if(sc.isOK()){
                break;
            }
            else{
                System.err.println(
                    String.format("!!! ERROR !!! swerve module: failed to update measurement from abs encoder for module {}: status code {}: description {}", mName, sc.getName(), sc.getDescription())
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


    public Angle GetTurnPosition() {
        return Units.Radians.of( mTurnEncoder.getPosition() );
    }

    public Angle GetTurnAbsPosition(){
        Angle position = Math.abs(
            Math.floorMod(
                mTurnAbsPositionSignal.getValue()
                + mTurnAbsEncoder
                + 180,
                360
            )
        ) - 180;
    }

    public Angle GetTurnAbsPositionRaw(){
        Angle position = mTurnAbsPositionSignal.getValue();
        return position;
    }

    public SwerveModulePosition GetPosition(){
        return new SwerveModulePosition(
            mDriveEncoder.getPosition(),
            Rotation2d.fromRadians(GetTurnPosition().in(Units.Radians))
        );
    }

    public SwerveModuleState GetState() {
        return new SwerveModuleState(
            mDriveEncoder.getVelocity(),
            Rotation2d.fromRadians(GetTurnPosition().in(Units.Radians))
        );
    }

    public void UpdateDashboard() {
        // TODO:
    }


    public void SetDesiredState(
        SwerveModuleState desiredState
    ) {
        Rotation2d encoderRotation = new Rotation2d( GetTurnPosition() );

        // opitmize the reference state to avoid spinning further than 90 deg

        SwerveModuleState targetState = SwerveModuleState.optimize(
            desiredState, encoderRotation
        );

        //  scale speed by cosine of angel error this scales down movement
        //  perpendicular to the desired direction of travel that can occur when
        //  modules change directions. this results in smoother driving

        // FIXME:
        // I do not know how rotation2d works

        targetState.speedMetersPerSecond *= (encoderRotation.minus(targetState.angle)).getCos();

        // Set drive speed percent.

        // FIXME:

        double drivePercent = Math.clamp(
            (targetState.speedMetersPerSecond / Constants.kMaxDriveSpeed).value(),
            -1.0, 1.0
        );

        mDriveMotor.set(drivePercent);

        // this has been marked for removel

        mTurnPid.setReference(
            targetState.angle.getRadians(),
            com.revrobotics.ControlType.kPosition
        );

    }

    public void SetTrunBreak(Boolean isEnabled){
        if(isEnabled){
            mTurnMotor.setIdleMode(revrobotics.CANSparkMax.IdleMode.kBrake);
        }
        else{
            mTurnMotor.setIdleMode(revrobotics.CANSparkMax.IdleMode.kCoast);
        }
    }

    public void ResetTurnPosition() {
        mTurnMotor.SetPosition(GetTurnAbsPosition());
    }

    public final class PidConfig {
        public double kP = 0.0;
        public double kI = 0.0;
        public double KD = 0.0;
    }
}