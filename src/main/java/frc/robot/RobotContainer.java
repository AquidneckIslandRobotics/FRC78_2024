// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.*;
import frc.robot.Systems.Chassis.*;
import frc.robot.Systems.Elevator;

public class RobotContainer {
  private Chassis m_chassis;
  private Elevator m_Elevator;
  private XboxController m_driveController;
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    m_chassis = new Chassis();

    CANSparkMax elevatorMotor = new CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless);
    m_Elevator = new Elevator(elevatorMotor);

    m_driveController = new XboxController(0);

    m_chassis.setDefaultCommand(
        new Drive(
            m_chassis,
            m_driveController::getLeftY,
            m_driveController::getLeftX,
            m_driveController::getRightX,
            m_driveController::getLeftTriggerAxis,
            m_driveController::getRightTriggerAxis,
            m_driveController::getYButton,
            m_driveController::getBButton,
            m_driveController::getAButton,
            m_driveController::getXButton));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("AutoMode", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    new Trigger(m_driveController::getStartButton)
        .onTrue(new InstantCommand(() -> m_chassis.resetPose(new Pose2d())));

    new Trigger(m_driveController::getRightBumper).onTrue(m_Elevator.moveToMiddle());

    new Trigger(m_driveController::getAButton)
        .whileTrue(m_Elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    new Trigger(m_driveController::getBButton)
        .whileTrue(m_Elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    new Trigger(m_driveController::getXButton)
        .whileTrue(m_Elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
    new Trigger(m_driveController::getYButton)
        .whileTrue(m_Elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
