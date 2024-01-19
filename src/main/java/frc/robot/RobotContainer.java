// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Systems.Elevator;
import frc.robot.Systems.Chassis.*;
import frc.robot.Commands.*;

public class RobotContainer {
  private Chassis m_chassis;
  private Elevator m_Elevator;
  private XboxController m_driveController;
  private XboxController m_testController;
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    m_chassis = new Chassis();

    m_Elevator = new Elevator();
  

    m_driveController = new XboxController(0);

    m_testController = new XboxController(5);

    m_chassis.setDefaultCommand(new Drive(
      m_chassis,
      m_driveController::getLeftY,
      m_driveController::getLeftX,
      m_driveController::getRightX,
      m_driveController::getLeftTriggerAxis,
      m_driveController::getRightTriggerAxis,
      m_driveController::getYButton,
      m_driveController::getBButton,
      m_driveController::getAButton,
      m_driveController::getXButton
      ));

      autoChooser = AutoBuilder.buildAutoChooser();

      SmartDashboard.putData("AutoMode", autoChooser);

      configureBindings();
  }

  private void configureBindings() {
    new Trigger(m_driveController::getStartButton).onTrue(new InstantCommand(() -> m_chassis.resetPose(new Pose2d())));

    new Trigger(m_testController::getYButton).whileTrue(new SetElevator(m_Elevator, -0.1)).onFalse(new SetElevator(m_Elevator, 0));
    new Trigger(m_testController::getAButton).whileTrue(new SetElevator(m_Elevator, 0.1)).onFalse(new SetElevator(m_Elevator, 0));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}