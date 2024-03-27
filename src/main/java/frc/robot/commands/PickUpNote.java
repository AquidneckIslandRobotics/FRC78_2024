package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class PickUpNote extends ParallelDeadlineGroup {
  public PickUpNote(Feeder feeder, Intake intake, Wrist wrist) {
    super(feeder.intake(), intake.intakeCommand(), wrist.setToTargetCmd(55));
  }
}
