package frc.robot.subsystems.drive;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

public class SwerveModule {
    String mName;

    CANSparkMax mDriveMotor;
    SparkPIDController mDrivePid;
    RelativeEncoder mDriveEncoder;

    CANSparkMax mTurnMotor;
    SparkPIDController mTurnPid;
    RelativeEncoder mTurnEncoder;

    CANcoder mTurnAbsEncoder;
    StatusSignal<Double> mTurnAbsPositionSignal;

    public static final Measure<Velocity<Angle>> kModuleMaxAngularVelocity = Units.RotationsPerSecond.of(0.5);

    public static final Measure<Velocity<Velocity<Angle>>> kModuleMaxAnguularAcceleration = Units.RotationsPerSecond
            .per(Units.Second).of(1.0);

    public SwerveModule(int driveMotorCan,
            int turnMotorCan,
            int turnAbsEncoderCan,
            Measure<Angle> absEncoderOffset,
            PidConfig turnConfig,
            String name
    ) 
    
    {
        mName = name;

        mDriveMotor = new CANSparkMax(driveMotorCan, MotorType.kBrushless);
        mDrivePid = mDriveMotor.getPIDController();
        mDriveEncoder = mDriveMotor.getEncoder();

        mTurnMotor = new CANSparkMax(turnMotorCan, MotorType.kBrushless);
        mTurnPid = mTurnMotor.getPIDController();
        mTurnEncoder = mTurnMotor.getEncoder();

        mTurnAbsEncoder = new CANcoder(turnAbsEncoderCan);
        mTurnAbsPositionSignal = mTurnAbsEncoder.getAbsolutePosition();
        
        // names are diffrent ill try my best to copy! -Weston Justice
        mTurnMotor.setInverted(true);

        // Set coast mode so we can straighten the wheels at start of match
        // Will switch to brake mode later

        mTurnMotor.setIdleMode(IdleMode.kCoast);
        mTurnMotor.setSmartCurrentLimit(30);

        CANcoderConfiguration configCanCoder = new CANcoderConfiguration();
        MagnetSensorConfigs configMagnetSensor = new MagnetSensorConfigs();
        configMagnetSensor
            .withAbsoluteSensorRange(com.ctre.phoenix6.signals.AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
            .withSensorDirection(com.ctre.phoenix6.signals.SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(0.0);

        configCanCoder.withMagnetSensor(configMagnetSensor);
        
        while (true){
            // will fix SOON 
            com.ctre.phoenix6.StatusCode sc = mTurnAbsEncoder.getConfigurator().apply(configMagnetSensor, 5_s)

            if(sc.isOK()){
                break;
            }
            else{
                fmt.println("!!! ERROR !!! swerve module: failed to apply config to abs encoder for module ${}: status code ${}: description ${}")
            }
        }
    }

    public final class PidConfig {
        public double kP = 0.0;
        public double kI = 0.0;
        public double KD = 0.0;
    }
}
