// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private final TalonFX feedMotor;

  private final HardwareLimitSwitchConfigs INTAKE_CONFIG =
      new HardwareLimitSwitchConfigs().withForwardLimitEnable(true);

  private final HardwareLimitSwitchConfigs SHOOT_CONFIG =
      new HardwareLimitSwitchConfigs().withForwardLimitEnable(false);

  /** Creates a new Feed. */
  public Feeder(int feedID) {
    feedMotor = new TalonFX(feedID);
    feedMotor.getForwardLimit().setUpdateFrequency(100);

    feedMotor.optimizeBusUtilization();

    SmartDashboard.putData(this);

    // If no commands are using this subsystem, stop the motor

  }

  /**
   * Intakes the note, stopping it when the beam-break sensor is tripped. Do not use this for
   * shooting, since the beam-brake sensor will prevent motor from moving
   */
  public Command intake() {
    return new FunctionalCommand(
            () -> {},
            () -> {},
            (interrupted) -> {},
            () -> feedMotor.getConfigurator().apply(INTAKE_CONFIG).isOK())
        .andThen(
            startEnd(() -> feedMotor.set(.85), () -> feedMotor.set(0))
                .until(
                    () ->
                        feedMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround))
        .andThen(
            new FunctionalCommand(
                () -> {},
                () -> {},
                (interrupted) -> {},
                () -> feedMotor.getConfigurator().apply(SHOOT_CONFIG).isOK()))
        .withName("Intake");
  }

  public Command outtake() {
    return startEnd(() -> feedMotor.set(-.8), () -> feedMotor.set(0)).withName("Outtake");
  }

  /**
   * Shoots the note, stopping it when the beam-break sensor is tripped. Do not use this for
   * shooting, since the beam-brake sensor will prevent motor from moving
   */
  public Command shoot() {
    return new FunctionalCommand(
            () -> {},
            () -> {},
            (interrupted) -> {},
            () -> feedMotor.getConfigurator().apply(SHOOT_CONFIG).isOK())
        .andThen(
            startEnd(() -> feedMotor.set(1), () -> feedMotor.set(0))
                .until(() -> feedMotor.getForwardLimit().getValue() == ForwardLimitValue.Open))
        .withName("Shoot");
  }

  public boolean isNoteQueued() {
    return feedMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("feeder/note", isNoteQueued());
  }
}
