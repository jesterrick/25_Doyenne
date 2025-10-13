package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakeStop extends InstantCommand {
  public IntakeStop(Intake intake) {
    super(intake::stopIntake, intake);
  }
}
