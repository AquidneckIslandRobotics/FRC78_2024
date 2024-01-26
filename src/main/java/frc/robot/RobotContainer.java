// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Systems.Chassis.*;
import frc.robot.Commands.*;
import frc.robot.Constants.RobotConstants;

public class RobotContainer {
  private Chassis m_chassis;
  private XboxController m_driveController;
  private PhotonCamera m_ATCamera;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    m_driveController = new XboxController(0);

    m_ATCamera = new PhotonCamera(RobotConstants.AT_CAMERA_NAME);
    PortForwarder.add(5800, "photonvision.local", 5800);

    m_chassis = new Chassis(m_ATCamera);

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
    new Trigger(m_driveController::getRightBumper).whileTrue(new OrbitalTarget(
      m_chassis,
      m_driveController::getLeftY,
      m_driveController::getLeftX,
      m_driveController::getRightX,
      m_driveController::getLeftTriggerAxis,
      m_driveController::getRightTriggerAxis
    ));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}