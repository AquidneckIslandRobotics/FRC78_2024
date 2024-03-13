package frc.robot.test_board;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Feedback;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {

  Feedback feedback = new Feedback(1);

  public Robot() {}

  public void robotInit() {
    RobotModeTriggers.disabled().whileTrue(feedback.showNoteAlignment().ignoringDisable(true));
  }

  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
