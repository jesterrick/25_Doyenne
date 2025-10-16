// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.constants.ElevatorConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorGoToStop extends Command {

  private final Elevator elevator;
  private final double setpoint;
  private final boolean shouldFinish;
  
  // Constructor that finishes when target is reached
  public ElevatorGoToStop(Elevator elevator, int stopIndex) {
    this(elevator, stopIndex, true);
  }
  
  // Constructor with option to run continuously or finish at target
  public ElevatorGoToStop(Elevator elevator, int stopIndex, boolean shouldFinish) {
    this.elevator = elevator;
    this.setpoint = ElevatorConstants.kStopPositions[stopIndex];
    this.shouldFinish = shouldFinish;
    addRequirements(this.elevator);    
  }

  // Called when the command is initially scheduled.
  // We set the target height and the periodic method in the Elevator subsystem will make sure we get there
  @Override
  public void initialize() {
    this.elevator.setTargetMeters(this.setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
    // Nothing needed here - the elevator subsystem handles the control loop
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Optionally stop the elevator when command ends
    // this.elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldFinish && elevator.atTarget();
  }
}
