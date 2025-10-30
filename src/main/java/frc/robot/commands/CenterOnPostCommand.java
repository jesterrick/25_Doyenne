// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.constants.VisionConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CenterOnPostCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final Limelight m_limelight;

  /** Creates a new CenterOnPostCommand. */
  public CenterOnPostCommand(Limelight limelt, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_limelight = limelt;
    this.m_driveSubsystem = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.setLEDMode(3); // Turn on LEDs
    m_limelight.setPipeline(VisionConstants.POST_PIPELINE); // Switch to post-detection pipeline
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_limelight.hasTarget()) {
      double tx = m_limelight.getTargetX(); // Horizontal offset

      // Calculate strafe power (proportional control)
      double drivePower = -tx * VisionConstants.CENTER_KP;

      // Add minimum power to overcome friction
      if (Math.abs(drivePower) > 0 && Math.abs(drivePower) < VisionConstants.CENTER_MIN_POWER) {
        drivePower = Math.copySign(VisionConstants.CENTER_MIN_POWER, drivePower);
      }

      // Clamp power to max
      drivePower = Math.max(-VisionConstants.CENTER_MAX_POWER,
          Math.min(VisionConstants.CENTER_MAX_POWER, drivePower));

      // Drive - adjust based on your drivetrain type
      // For mecanum/swerve: strafe sideways
      m_driveSubsystem.drive(drivePower, 0, 0, false); // (forward, strafe, rotate)

      // For tank/differential: turn in place
      // m_drive.arcadeDrive(0, strafePower);

    } else {
      m_driveSubsystem.stop(); // Stop if no target
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Finish when centered within tolerance
    return m_limelight.hasTarget() &&
        Math.abs(m_limelight.getTargetX()) < VisionConstants.CENTER_TOLERANCE;
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop();
    m_limelight.setLEDMode(1); // Turn off LEDs
  }
}
