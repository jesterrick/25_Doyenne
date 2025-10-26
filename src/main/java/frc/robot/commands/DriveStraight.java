// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraight extends Command {
  private final DriveSubsystem m_drive;
  private final double m_speed;

  /**
   * Creates a new DriveStraight command.
   * 
   * @param drive The drive subsystem
   * @param speed Speed to drive forward (0.0 to 1.0)
   */
  public DriveStraight(DriveSubsystem drive, double speed) {
    m_drive = drive;
    m_speed = speed;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Nothing needed
  }

  @Override
  public void execute() {
    // Drive forward at the specified speed
    m_drive.drive(m_speed, 0, 0, false);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when command ends
    m_drive.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    // This command runs until interrupted or timed out
    return false;
  }
}