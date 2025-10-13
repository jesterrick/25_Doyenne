// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorJoystick extends Command {
  private final Elevator elevator;
  private final DoubleSupplier ySpeedSupplier;

  /** Creates a new ElevatorJoystick. */
  public ElevatorJoystick(Elevator elevator, DoubleSupplier ySupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.ySpeedSupplier = ySupplier;
    addRequirements(this.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ySpeed = this.ySpeedSupplier.getAsDouble() * ElevatorConstants.kMaxElevatorSpeed;    
    this.elevator.setPercentOutput(MathUtil.applyDeadband(ySpeed, 0.05));
    this.elevator.getPositionMeters(); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}
