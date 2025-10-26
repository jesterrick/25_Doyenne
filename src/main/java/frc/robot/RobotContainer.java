// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveStraight;
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
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeRotator;
import frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
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
    private final Intake m_Intake = new Intake();
    private final IntakeRotator m_IntakeRotator = new IntakeRotator();
    private final Outtake m_outtake = new Outtake();
    private final Elevator m_elevator = new Elevator();
    public DriveSubsystem m_robotDrive = new DriveSubsystem(m_elevator);

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
        // 2. After a short wait time, run the intake motor at eject speed to push the
        // game piece into the outtake
        m_intakeToOuttakeButton.onTrue(
                new ParallelDeadlineGroup(
                        new OuttakeReceive(this.m_outtake, OuttakeConstants.kOuttakeReceiveSpeed)
                                .withTimeout(OuttakeConstants.kOuttakeReceiveTime),
                        new SequentialCommandGroup(
                                new WaitCommand(OuttakeConstants.inOutWaitTime),
                                new IntakeEject(this.m_Intake,
                                        IntakeConstants.kIntakeEjectSpeed)
                                        .withTimeout(IntakeConstants.kIntakeEjectDuration))));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new DriveStraight(m_robotDrive, DriveConstants.kMaxAutonomousSpeed).withTimeout(3.0);
    }
}