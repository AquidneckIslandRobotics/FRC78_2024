// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
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

    intakeTop.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 65535);
    intakeTop.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
    intakeTop.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
    intakeTop.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    intakeTop.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    intakeTop.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    intakeTop.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
    intakeTop.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 65535);
    intakeBottom.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 65535);
    intakeBottom.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
    intakeBottom.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
    intakeBottom.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    intakeBottom.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    intakeBottom.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    intakeBottom.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
    intakeBottom.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 65535);

    this.intakeSpeed = intakeSpeed;
    this.outtakeSpeed = outtakeSpeed;
  }

  /* intake speed is same for top and bottom */
  public void setIntake(double speed) {
    intakeTop.set(speed);
    intakeBottom.set(speed);
  }

  public Command intakeCommand() {
    return this.startEnd(() -> this.setIntake(intakeSpeed), () -> this.setIntake(0));
  }

  public Command outtakeCommand() {
    return this.startEnd(() -> this.setIntake(outtakeSpeed), () -> this.setIntake(0));
  }
}
