// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.autocommands.IntakeCommand;
import frc.robot.commands.autocommands.ShootCommand;
import frc.robot.commands.autocommands.DriveCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.HashMap;
import java.util.List;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final ShooterSubsystem m_shooter = ShooterSubsystem.getInstance();
        private final IntakeSubsystem m_intake = IntakeSubsystem.getInstance();

        // The driver's controller
        // Joystick m_driverController = new
        // Joystick(OIConstants.kDriverControllerPort);
        public static final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
        public static final XboxController m_operator = new XboxController(OIConstants.kOperatorControllerPort);

        // Auto Chooser for Autos
        private final SendableChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                NamedCommands.registerCommand("ShootRun", new ShootCommand(m_shooter, "run"));
                NamedCommands.registerCommand("IntakeTertiaryDelay", new IntakeCommand(m_intake, "delay"));
                NamedCommands.registerCommand("ShootStop", new ShootCommand(m_shooter, "stop"));
                NamedCommands.registerCommand("IntakeStop", new IntakeCommand(m_intake, "stop"));
                NamedCommands.registerCommand("IntakeRun", new IntakeCommand(m_intake, "run"));
                NamedCommands.registerCommand("ZeroGyro", new DriveCommand(m_robotDrive, "zero"));
                // Configure the button bindings
                configureButtonBindings();

                // Set auto chooser to Auto Builder
                autoChooser = AutoBuilder.buildAutoChooser();

                // Smart Dashboard
                SmartDashboard.putData("Auto Chooser", autoChooser);

                // Configure default commands
                m_robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                -MathUtil.applyDeadband(m_driverController.getLeftY(), // y
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getLeftX(), // x
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                                OIConstants.kDriveDeadband),
                                                                true, true),
                                                m_robotDrive));

        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {
                // Initialize buttons
                final JoystickButton rbXbox = new JoystickButton(m_driverController,
                                XboxController.Button.kRightBumper.value);
                final JoystickButton bXbox = new JoystickButton(m_driverController, XboxController.Button.kB.value);
                final JoystickButton aXbox = new JoystickButton(m_driverController, XboxController.Button.kA.value);
                final JoystickButton lbXbox = new JoystickButton(m_driverController,
                                XboxController.Button.kLeftBumper.value);
                final JoystickButton xXbox = new JoystickButton(m_driverController, XboxController.Button.kX.value);
                final JoystickButton yXbox = new JoystickButton(m_driverController, XboxController.Button.kY.value);

                final JoystickButton xFS = new JoystickButton(m_operator, XboxController.Button.kX.value);
                final JoystickButton yFS = new JoystickButton(m_operator, XboxController.Button.kY.value);
                final JoystickButton aFS = new JoystickButton(m_operator, XboxController.Button.kA.value);
                final JoystickButton bFS = new JoystickButton(m_operator, XboxController.Button.kB.value);
                final JoystickButton rbFS = new JoystickButton(m_operator, XboxController.Button.kRightBumper.value);
                final JoystickButton lbFS = new JoystickButton(m_operator, XboxController.Button.kLeftBumper.value);

                // new JoystickButton(m_driverController, Button.kCircle.value)
                xXbox.whileTrue(new RunCommand(
                                () -> m_robotDrive.setX(),
                                m_robotDrive));

                // Reset gyro heading to 0
                yXbox.whileTrue(new RunCommand( 
                        () -> m_robotDrive.zeroHeading(), m_robotDrive));

                // Right Bumper on Xbox//
                
                // // Start shooter on press
                // lbXbox.onTrue(new RunCommand(
                                //() -> ShooterSubsystem.getInstance().shootNormal(), ShooterSubsystem.getInstance()));
                // // Stop shooter on not pressed
                // lbXbox.onFalse(new RunCommand(
                               // () -> ShooterSubsystem.getInstance().stop(), ShooterSubsystem.getInstance()));
                // // Start 3rd intake on press
                // rbXbox.onTrue(new RunCommand(
                //                 () -> IntakeSubsystem.getInstance().tertiaryDelay(), IntakeSubsystem.getInstance()));
                // // Stop 3rd intake on not pressed
                // rbXbox.onFalse(new RunCommand(
                //                 () -> IntakeSubsystem.getInstance().stop(), IntakeSubsystem.getInstance()));

                // Left Bumper on Xbox//
                // Set x wheel orientation
                rbXbox.whileTrue(new RunCommand(
                                () -> m_robotDrive.setX(), m_robotDrive));
                // Start slow shooter on press
                rbXbox.onTrue(new RunCommand(
                                () -> ShooterSubsystem.getInstance().shootSlow(), ShooterSubsystem.getInstance()));
                // Stop slow shooter on not pressed
                rbXbox.onFalse(new RunCommand(
                                () -> ShooterSubsystem.getInstance().stop(), ShooterSubsystem.getInstance()));
                // Start 3rd intake on press
                rbXbox.onTrue(new RunCommand(
                                () -> IntakeSubsystem.getInstance().tertiaryFastDelay(), IntakeSubsystem.getInstance()));
                // Stop 3rd intake on not pressed
                rbXbox.onFalse(new RunCommand(
                                () -> IntakeSubsystem.getInstance().stop(), IntakeSubsystem.getInstance()));

                // X Button on Fight Stick//
                // Run intake on X press
                xFS.onTrue(new RunCommand(
                                () -> IntakeSubsystem.getInstance().mainIntake(), IntakeSubsystem.getInstance()));
                // Stop intake on X not pressed
                xFS.onFalse(new RunCommand(
                                () -> IntakeSubsystem.getInstance().stop(), IntakeSubsystem.getInstance()));

                // Y Button on Fight Stick//
                // Run outake on Y press
                yFS.onTrue(new RunCommand(
                                () -> IntakeSubsystem.getInstance().mainOutake(), IntakeSubsystem.getInstance()));
                // Stop intake on Y not pressed
                yFS.onFalse(new RunCommand(
                                () -> IntakeSubsystem.getInstance().stop(), IntakeSubsystem.getInstance()));

                // A Button on Fight Stick//
                // Run front intake on A press
                aFS.onTrue(new RunCommand(
                                () -> ShooterSubsystem.getInstance().frontIntake(), ShooterSubsystem.getInstance()));
                aFS.onTrue(new RunCommand(
                                () -> IntakeSubsystem.getInstance().tertiary(), IntakeSubsystem.getInstance()));
                aFS.onFalse(new RunCommand(
                                () -> ShooterSubsystem.getInstance().stop(), ShooterSubsystem.getInstance()));
                aFS.onFalse(new RunCommand(
                                () -> IntakeSubsystem.getInstance().stop(), IntakeSubsystem.getInstance()));

                // // RB and lb for climbing up and down //
                // rbFS.onTrue(new RunCommand(
                // () -> ClimberSubsystem.getInstance().climbUp(),
                // ClimberSubsystem.getInstance()));
                // rbFS.onFalse(new RunCommand(
                // () -> ClimberSubsystem.getInstance().stop(),
                // ClimberSubsystem.getInstance()));
                // lbFS.onTrue(new RunCommand(
                // () -> ClimberSubsystem.getInstance().climbDown(),
                // ClimberSubsystem.getInstance()));

                // lbFS.onFalse(new RunCommand(
                // () -> ClimberSubsystem.getInstance().stop(),
                // ClimberSubsystem.getInstance()));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */

        // PathPlannerPath path = PathPlannerPath.fromPathFile("Mid-Double Blue");

        // if (checkAlliance() == true) {
        // path = PathPlannerPath.fromPathFile("Mid-Double Red");
        // }

        // // Create a path following command using AutoBuilder. This will also trigger
        // event markers.
        // return AutoBuilder.followPath(path);

        // PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example Path", new
        // PathConstrains(4, 3));

        // // This is just an example event map. It would be better to have a constant,
        // global event map
        // // in your code that will be used by all path following commands.
        // HashMap<String, Command> eventMap = new HashMap<>();

        // eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        // eventMap.put("intakeDown", new IntakeDown());

        // FollowPathWithEvents command = new FollowPathWithEvents(
        // getPathFollowingCommand(examplePath),
        // examplePath.getMarkers(),
        // eventMap
        // );
        public Command getAutonomousCommand() {
                // ShooterSubsystem.getInstance().shootNormal();

                // IntakeSubsystem.getInstance().tertiaryDelay();
                // Timer.delay(1);
                // ShooterSubsystem.getInstance().stop();
                // IntakeSubsystem.getInstance().stop();
                // IntakeSubsystem.getInstance().mainIntake();

                // PathPlannerPath path = PathPlannerPath.fromPathFile("BackPath");
                // PathPlannerPath path2 = PathPlannerPath.fromPathFile("front");

                // return AutoBuilder.followPath(path2);

                return autoChooser.getSelected();

                // // Create config for trajectory
                // TrajectoryConfig config = new TrajectoryConfig()
                // AutoConstants.kMaxSpeedMetersPerSecond,
                // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // // Add kinematics to ensure max speed is actually obeyed
                // .setKinematics(DriveConstants.kDriveKinematics);

                // // An example trajectory to follow. All units in meters.
                // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // // Start at the origin facing the +X direction
                // new Pose2d(0, 0, new Rotation2d(0)),
                // // Pass through these two interior waypoints, making an 's' curve path
                // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // // End 3 meters straight ahead of where we started, facing forward
                // new Pose2d(3, 0, new Rotation2d(0)),
                // config);

                // var thetaController = new ProfiledPIDController(
                // AutoConstants.kPThetaController, 0, 0,
                // AutoConstants.kThetaControllerConstraints);
                // thetaController.enableContinuousInput(-Math.PI, Math.PI);

                // SwerveControllerCommand swerveControllerCommand = new
                // SwerveControllerCommand(
                // exampleTrajectory,
                // m_robotDrive::getPose, // Functional interface to feed supplier
                // DriveConstants.kDriveKinematics,

                // // Position controllers
                // new PIDController(AutoConstants.kPXController, 0, 0),
                // new PIDController(AutoConstants.kPYController, 0, 0),
                // thetaController,
                // m_robotDrive::setModuleStates,
                // m_robotDrive);

                // // Reset odometry to the starting pose of the trajectory.
                // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

                // // Run path following command, then stop at the end.
                // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
                // false, false));
        }
}
