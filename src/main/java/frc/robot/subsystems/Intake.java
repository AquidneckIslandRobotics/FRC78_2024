// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeTop;
  private CANSparkMax intakeBottom;

  private double intakeSpeed;
  private double outtakeSpeed;

  /** Creates a new Intake. */
  public Intake(int intakeTopId, int intakeBottomId, double intakeSpeed, double outtakeSpeed) {
    intakeTop = new CANSparkMax(intakeTopId, MotorType.kBrushless);
    intakeBottom = new CANSparkMax(intakeBottomId, MotorType.kBrushless);

    intakeTop.restoreFactoryDefaults();
    intakeBottom.restoreFactoryDefaults();

    this.intakeSpeed = intakeSpeed;
    this.outtakeSpeed = outtakeSpeed;
  }

  /* intake speed is same for top and bottom */
  public void intakeControl(double speed) {
    intakeTop.set(speed);
    intakeBottom.set(speed);
  }

  public Command intakeCommand() {
    return this.startEnd(() -> this.intakeControl(intakeSpeed), () -> this.intakeControl(0));
  }

  public Command outtakeCommand() {
    return this.startEnd(() -> this.intakeControl(outtakeSpeed), () -> this.intakeControl(0));
  }
}