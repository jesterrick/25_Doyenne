// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_xSupplier;
  private final DoubleSupplier m_ySupplier;
  private final DoubleSupplier m_rotSupplier;
  private final BooleanSupplier m_fieldRelativeSupplier;

  /**
   * Creates a new DriveCommand.
   *
   * @param drive                 The drive subsystem to use
   * @param xSupplier             Supplier for forward/backward movement (X axis)
   * @param ySupplier             Supplier for left/right movement (Y axis)
   * @param rotSupplier           Supplier for rotation (Z axis)
   * @param fieldRelativeSupplier Supplier for whether to use field-relative
   *                              control
   */
  public DriveCommand(DriveSubsystem drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
      DoubleSupplier rotSupplier, BooleanSupplier fieldRelativeSupplier) {
    m_drive = drive;
    m_xSupplier = xSupplier;
    m_ySupplier = ySupplier;
    m_rotSupplier = rotSupplier;
    m_fieldRelativeSupplier = fieldRelativeSupplier;

    addRequirements(drive);
  }

  @Override
  public void execute() {
    // Get joystick inputs and apply deadband
    double xSpeed = -MathUtil.applyDeadband(m_xSupplier.getAsDouble(), OIConstants.kDriveDeadband);
    double ySpeed = -MathUtil.applyDeadband(m_ySupplier.getAsDouble(), OIConstants.kDriveDeadband);
    double rot = -MathUtil.applyDeadband(m_rotSupplier.getAsDouble(), OIConstants.kDriveDeadband);

    // Drive the robot
    m_drive.drive(xSpeed, ySpeed, rot, m_fieldRelativeSupplier.getAsBoolean());
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drive when command ends
    m_drive.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    return false; // This command runs continuously
  }
}