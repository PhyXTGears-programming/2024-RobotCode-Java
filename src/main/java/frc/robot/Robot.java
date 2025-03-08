// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.commands.driveTeleopCommand.DriveTeleopCommand;
import frc.robot.lib.config.ConfigLoader;
import frc.robot.lib.config.ConfigTable;
import frc.robot.lib.config.TomlConfigLoader;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.vision.*;
import frc.robot.commands.visionDriveCommand.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    private Drivetrain kDrivetrain = null;

    private Vision kVisionSubsystem = null;

    private VisionAutoCommand kVisionAutoCommand = null;

    private VisionTeleopCommand mVisionTeleopCommand = null;

    private DriveTeleopCommand kDriveTeleopCommand = null;

    private XboxController kDriveController = new XboxController(0);

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {

        String path = Filesystem.getDeployDirectory().getAbsolutePath();

        ConfigLoader config = new TomlConfigLoader(path + "/config.toml");

        config.openConfig();

        Optional<ConfigTable> table = config.getTable("drivetrain");

        if (table.isEmpty()) {
            throw new Error("Unable to find config table 'drivetrain' ");
        }

        kDrivetrain = new Drivetrain(table.get());

        // init vision

        table = config.getTable("vision");

        if (table.isEmpty()) {
            throw new Error("Unable to find config table 'vision' ");
        }

        kVisionSubsystem = new Vision(table.get());
        kVisionAutoCommand = new VisionAutoCommand(kVisionSubsystem, kDrivetrain, 10, Units.Milliseconds.of(20.0));
        mVisionTeleopCommand = new VisionTeleopCommand(kVisionSubsystem, kDrivetrain, -1, Units.Milliseconds.of(20.0));
        
        kDriveTeleopCommand = new DriveTeleopCommand(kDrivetrain, kDriveController, Units.Milliseconds.of(20.0));

        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        SmartDashboard.putData(CommandScheduler.getInstance());

        // update the kVisionSubsytem
        kVisionSubsystem.updateLimelight();
        kVisionSubsystem.updateSmartDashboard();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different
     * autonomous modes using the dashboard. The sendable chooser code works with
     * the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
     * chooser code and
     * uncomment the getString line to get the auto name from the text box below the
     * Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure
     * below with additional strings. If using the SendableChooser make sure to add
     * them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        m_autoSelected = m_chooser.getSelected();
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        System.out.println("Auto selected: " + m_autoSelected);

        CommandScheduler.getInstance().schedule(
                kVisionAutoCommand);

    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        switch (m_autoSelected) {
            case kCustomAuto:
                // Put custom auto code here
                break;
            case kDefaultAuto:
            default:
                // Put default auto code here
                break;
        }
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        kDriveTeleopCommand.schedule();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        // code to handle swaping vison and teleop

        boolean leftBumper = kDriveController.getLeftBumperButtonPressed();
        boolean rightBumper = kDriveController.getRightBumperButtonPressed();

        if (leftBumper || rightBumper) {
            System.out.println("Driving to april tag...");

            mVisionTeleopCommand.setOffset(leftBumper ? Vision.Alignment.LEFT : Vision.Alignment.RIGHT);

            kDriveTeleopCommand.cancel();
            mVisionTeleopCommand.schedule();

        } else if (kDriveController.getLeftBumperButtonReleased() || kDriveController.getRightBumperButtonReleased()) {
            mVisionTeleopCommand.cancel();
            kDriveTeleopCommand.schedule();
            System.out.println("Cancel");

        }
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
