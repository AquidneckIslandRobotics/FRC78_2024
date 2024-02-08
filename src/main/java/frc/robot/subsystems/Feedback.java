// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feedback extends SubsystemBase {
  /** Creates a new Feedback. */
  private CANdle bracelet;

  public Feedback(int candleID) {
    bracelet = new CANdle(candleID);
  }

  public void red() {
    bracelet.setLEDs(255, 0, 0);
  }

  public void multi(Color color) {
    bracelet.setLEDs(
        ((int) (color.red * 255)), ((int) (color.green * 255)), ((int) (color.blue * 255)));
  }

  public void off() {
    bracelet.setLEDs(0, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

/*   Can use this to add feedback light to buttons
 *   new Trigger(m_driveController::getStartButton).onTrue(new InstantCommand(() -> m_chassis.resetPose(new Pose2d())));
 *   new Trigger(m_driveController::getBButton).whileTrue(Commands.startEnd(m_feedback::red, m_feedback::off, m_feedback));
 *   new Trigger(m_driveController::getXButton).whileTrue(Commands.startEnd(()->m_feedback.multi(new Color(186,7,162)), m_feedback::off, m_feedback));
 *   new Trigger(m_driveController::getYButton).whileTrue(Commands.startEnd(()->m_feedback.multi(Color.fromHSV(316,47,10)), m_feedback::off, m_feedback));
 *   new Trigger(m_driveController::getAButton).whileTrue(Commands.startEnd(()->m_feedback.multi(new Color(0,255,0)), m_feedback::off, m_
 */
