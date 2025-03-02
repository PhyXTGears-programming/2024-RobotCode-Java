package frc.robot.commands.driveTeleopCommand;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import frc.robot.subsystems.drive.*;
import frc.robot.Constants;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;


public class DriveTeleopCommand extends Command {
    
    private Time mRobotPeriod;
    
    private Drivetrain mDrivetrain;

    private XboxController mDriveController;

    private SlewRateLimiter mForwardSpeedLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter mStrafeSpeedLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter mTurnSpeedLimiter = new SlewRateLimiter(3.0);

    private final double JOYSTICK_DEADZONE = 0.2;

    public DriveTeleopCommand(
        Drivetrain drivetrain,
        XboxController driverController,
        Time period
    ) {
        addRequirements(drivetrain);

        mDriveController = driverController;
        mDrivetrain = drivetrain;
        mRobotPeriod = period;

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        mDrivetrain.updateOdometry();
        //compresses the range of the driving speed to be within the max speed and
        //the minimum. but have the normal speed be the default if no trigger is 
        //being pressed (so both register 0)
        //
        //NOTE no trigger takes priority of the other so if both pressed they will cancel each other 

        if (mDriveController.getBButtonPressed()) {
            mDrivetrain.zeroGyro();
            System.out.println("Reseting Gyro");
        }

        double leftTrigger = mDriveController.getLeftTriggerAxis(); // Range [0.0..1.0]
        double rightTrigger = mDriveController.getRightTriggerAxis(); // Range [0.0..1.0]

        LinearVelocity driveReduce = Constants.kNormalDriveSpeed.minus(Constants.kSlowDriveSpeed).times(leftTrigger);
        
        LinearVelocity driveGain   = Constants.kFastDriveSpeed.minus(Constants.kNormalDriveSpeed).times(rightTrigger);

        LinearVelocity driveSpeedFactor = Constants.kNormalDriveSpeed.plus(driveGain).minus(driveReduce);

        AngularVelocity turnReduce = (Constants.kNormalTurnSpeed.minus(Constants.kSlowTurnSpeed)).times(leftTrigger);
        AngularVelocity turnGain   = (Constants.kFastTurnSpeed.minus(Constants.kNormalTurnSpeed)).times(rightTrigger);

        AngularVelocity turnSpeedFactor = Constants.kNormalTurnSpeed.plus(turnGain).minus(turnReduce);

        // the rotation limit is there in case the driver does not want to spin as fast while driving
        final LinearVelocity forwardSpeed = driveSpeedFactor.times(
            mForwardSpeedLimiter.calculate(applyDeadband(
                -mDriveController.getLeftY(),
                JOYSTICK_DEADZONE))
        );

        final LinearVelocity strafeSpeed = driveSpeedFactor.times(
            mStrafeSpeedLimiter.calculate(applyDeadband(
                -mDriveController.getLeftX(),
                JOYSTICK_DEADZONE))
        );
  
        final AngularVelocity turnSpeed = turnSpeedFactor.times(
            mTurnSpeedLimiter.calculate(applyDeadband(
                -mDriveController.getRightX(),
                JOYSTICK_DEADZONE))
        );

        mDrivetrain.Drive(
            forwardSpeed,
            strafeSpeed,
            turnSpeed,
            mDrivetrain.IsFieldOriented(),
            mRobotPeriod
        );

    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drive motors!
        mDrivetrain.Drive(
            Units.MetersPerSecond.of(0.0),
            Units.MetersPerSecond.of(0.0),
            Units.RadiansPerSecond.of(0.0),
            true,
            mRobotPeriod
        );
    }

    @Override
    public boolean isFinished() {
        return false; //dont end because then we wont be able to drive
    }

}
