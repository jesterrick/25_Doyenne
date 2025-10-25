// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import org.json.simple.parser.ParseException;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ElevatorGoToStop;
import frc.robot.commands.ElevatorJoystick;
import frc.robot.commands.IntakeReceive;
import frc.robot.commands.IntakeRotateDown;
import frc.robot.commands.IntakeRotateUp;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.OuttakeEject;
import frc.robot.commands.OuttakeReceive;
import frc.robot.commands.IntakeEject;
import frc.robot.constants.OIConstants;
import frc.robot.constants.OuttakeConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeRotator;
import frc.robot.subsystems.Elevator;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    public DriveSubsystem m_robotDrive = new DriveSubsystem();
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

    // Buttons
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
                new DriveCommand(
                        m_robotDrive,
                        () -> m_driverJoystick.getY(),
                        () -> m_driverJoystick.getX(),
                        () -> m_driverJoystick.getZ(),
                        () -> false));
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

    m_elevator0Button.onTrue(new ElevatorGoToStop(m_elevator, 0));
    m_elevator1Button.onTrue(new ElevatorGoToStop(m_elevator, 1));
    m_elevator2Button.onTrue(new ElevatorGoToStop(m_elevator, 2));
    m_elevator3Button.onTrue(new ElevatorGoToStop(m_elevator, 3));
    m_elevator4Button.onTrue(new ElevatorGoToStop(m_elevator, 4));

    this.m_elevator.setDefaultCommand(
        new ElevatorJoystick(
            this.m_elevator,
            () -> -this.m_operatorJoystick.getY()));

    m_outtakeEjectButton.onTrue(new OuttakeEject(m_outtake, OuttakeConstants.kOuttakeEjectSpeed)
        .withTimeout(OuttakeConstants.kOuttakeEjectTime));

    // while the button is being held
    // 1. Lift the elevator so its out of the way
    // 2. turn on the intake while lowering the intake at the same time
    m_intakeReceiveButton.whileTrue(
        new SequentialCommandGroup(
            new ElevatorGoToStop(m_elevator, 1).withTimeout(1),
            new ParallelCommandGroup(
                new IntakeRotateDown(this.m_IntakeRotator),
                new IntakeReceive(this.m_Intake, IntakeConstants.kIntakeMotorSpeed))));
    // when the button is released it will perform the following tasks in the order
    // listed
    // 1. Stop the intake
    // 2. Raise the intake
    // 3. Lower the elevator back to the starting position
    m_intakeReceiveButton.onFalse(
        new SequentialCommandGroup(
            new IntakeStop(this.m_Intake),
            new IntakeRotateUp(m_IntakeRotator).withTimeout(2),
            new ElevatorGoToStop(m_elevator, 0)));

    // when this button is pressed it will
    // 1. Run the Outtake motor at receive speed
    // 2. After a short wait time, run the intake motor at eject speed to push the game piece into the outtake
    m_intakeToOuttakeButton.onTrue(
        new ParallelDeadlineGroup(
            new OuttakeReceive(this.m_outtake, OuttakeConstants.kOuttakeReceiveSpeed)
                .withTimeout(OuttakeConstants.kOuttakeReceiveTime),
            new SequentialCommandGroup(
                new WaitCommand(OuttakeConstants.inOutWaitTime),
                new IntakeEject(this.m_Intake,
                    IntakeConstants.kIntakeEjectSpeed)
                    .withTimeout(IntakeConstants.kIntakeEjectDuration))
            ));
  }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        PathPlannerPath path;
    
        try {
            // Attempt to load the PathPlanner path
            path = PathPlannerPath.fromPathFile("Example Path");
        } catch (IOException | ParseException | FileVersionException e) {
            // Print error message to console
            System.err.println("🚨 Error: Failed to load PathPlanner path! Reason: " + e.getMessage());
            e.printStackTrace();  // Print full error for debugging
            
            // Return a safe fallback command (e.g., do nothing)
            return new Command() {
                @Override
                public void initialize() {
                    System.out.println("⚠️ Running fallback autonomous: No path loaded.");
                }
            };
        }
    
        // Get the first waypoint's position
        Translation2d startPosition = path.getPoint(0).position;
    
        // Convert it to a full Pose2d by adding a default rotation
        Pose2d startingPose = new Pose2d(startPosition, new Rotation2d(0));
    
        // Reset odometry to match the path's starting pose
        m_robotDrive.resetOdometry(startingPose);
    
        // Use AutoBuilder to follow the path
        Command pathCommand = AutoBuilder.followPath(path);
    
        // Run the command and stop at the end
        return AutoBuilder.followPath(path)
            .andThen(new InstantCommand(() -> m_robotDrive.drive(0,0,0,false), m_robotDrive)); // ✅ Ensures the robot stops at the end
    }
}
