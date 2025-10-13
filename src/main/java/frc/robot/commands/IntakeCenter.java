package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCenter extends Command {
  private final Intake intake;
  private final double speed;

  public IntakeCenter(Intake intake, double speed) {
    this.intake = intake;
    this.speed = speed;
    addRequirements(this.intake);
  }

  @Override
  public void execute() {
    this.intake.runCentering(this.speed);
  }

  @Override
  public void end(boolean interrupted) {
    this.intake.stopCentering();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
