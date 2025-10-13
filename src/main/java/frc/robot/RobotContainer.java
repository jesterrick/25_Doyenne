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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ElevatorGoToStop;
import frc.robot.commands.IntakeReceive;
import frc.robot.commands.IntakeRotate;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.OuttakeEject;
import frc.robot.commands.IntakeCenter;
import frc.robot.commands.IntakeEject;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;
import frc.robot.constants.OuttakeConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.IntakeRotatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeRotator;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final Intake m_Intake = new Intake();
    private final IntakeRotator m_IntakeRotator = new IntakeRotator();
    private final Outtake m_outtake = new Outtake();
    private final Elevator m_elevator = new Elevator();

    // The driver's controller
    Joystick m_driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);
    Joystick m_operatorJoystick = new Joystick(OIConstants.kOperatorJoystickPort);

    JoystickButton m_intakeReceiveButton = new JoystickButton(m_operatorJoystick, OIConstants.kIntakeReceiveButton);
    JoystickButton m_intakeToOuttakeButton = new JoystickButton(m_operatorJoystick, OIConstants.kIntakeToOuttakeButton);
    JoystickButton m_outtakeEjectButton = new JoystickButton(m_operatorJoystick, OIConstants.kOuttakeEjectButton);
    JoystickButton m_elevator0Button = new JoystickButton(m_operatorJoystick, OIConstants.kElevatorPositionButton0);
    JoystickButton m_elevator1Button = new JoystickButton(m_operatorJoystick, OIConstants.kElevatorPositionButton1);
    JoystickButton m_elevator2Button = new JoystickButton(m_operatorJoystick, OIConstants.kElevatorPositionButton2);
    JoystickButton m_elevator3Button = new JoystickButton(m_operatorJoystick, OIConstants.kElevatorPositionButton3);
    JoystickButton m_elevator4Button = new JoystickButton(m_operatorJoystick, OIConstants.kElevatorPositionButton4);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -MathUtil.applyDeadband(m_driverJoystick.getX(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverJoystick.getY(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverJoystick.getZ(), OIConstants.kDriveDeadband),
                                true),
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
        new JoystickButton(m_driverJoystick, Button.kR1.value)
                .whileTrue(new RunCommand(
                        () -> m_robotDrive.setX(),
                        m_robotDrive));

        m_elevator0Button.onTrue(new ElevatorGoToStop(m_elevator, 0));
        m_elevator1Button.onTrue(new ElevatorGoToStop(m_elevator, 1));
        m_elevator2Button.onTrue(new ElevatorGoToStop(m_elevator, 2));
        m_elevator3Button.onTrue(new ElevatorGoToStop(m_elevator, 3));
        m_elevator4Button.onTrue(new ElevatorGoToStop(m_elevator, 4));

        m_outtakeEjectButton.onTrue(new OuttakeEject(m_outtake, OuttakeConstants.kOuttakeMotorSpeed)
                .withTimeout(OuttakeConstants.kOuttakeEjectTime));

        // while the button is being held, the intake assembly will lower and the intake
        // motors will run
        // in the direction to pull the coral into the intake
        // these actions occur at the same time
        m_intakeReceiveButton.whileTrue(
                new ParallelCommandGroup(
                        new IntakeRotate(this.m_IntakeRotator, IntakeRotatorConstants.kIntakeRotatorMotorDown),
                        new IntakeReceive(this.m_Intake, IntakeConstants.kIntakeMotorSpeed)));
        // when the button is released it will perform the following tasks in the order
        // listed
        // 1. Stop the intake
        // 2. Raise the intake and run the centering motor for kCenteringDuration
        // seconds
        // 3. Run the intake motor backwards for kReverseDuration to eject the coral
        // 4. Stop the intake
        m_intakeReceiveButton.onFalse(
                new SequentialCommandGroup(
                        new IntakeStop(this.m_Intake),
                        new ParallelCommandGroup(
                                new IntakeRotate(this.m_IntakeRotator, IntakeRotatorConstants.kIntakeRotatorMotorUp),
                                new IntakeCenter(this.m_Intake,
                                        IntakeConstants.kIntakeMotorSpeed)
                                        .withTimeout(IntakeConstants.kIntakeCenterDuration))));

        // when this button is pressed it will run the intake motor to eject the coral
        // and at the same time run the outtake motor to accept the coral
        // both wiill run for the time set by kIntakeEjectDuration
        m_intakeToOuttakeButton.onTrue(
                new ParallelCommandGroup(
                        new IntakeEject(this.m_Intake, IntakeConstants.kIntakeMotorSpeed)
                                .withTimeout(IntakeConstants.kIntakeEjectDuration),
                        new OuttakeEject(this.m_outtake, OuttakeConstants.kOuttakeMotorSpeed)
                                .withTimeout(IntakeConstants.kIntakeEjectDuration)));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        var thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                m_robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}
