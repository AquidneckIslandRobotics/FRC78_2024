// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.Util;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  private CANSparkMax wristNeo;
  private AbsoluteEncoder encoder;
  private double stowPos = 55;
  private double target = 0;

  /** Creates a new Wrist. */
  public Wrist(int WRIST_ID, float WRIST_HIGH_LIM, float WRIST_LOW_LIM) {
    wristNeo = new CANSparkMax(WRIST_ID, MotorType.kBrushless);

    wristNeo.restoreFactoryDefaults();

    wristNeo.setIdleMode(IdleMode.kBrake);

    encoder = wristNeo.getAbsoluteEncoder(Type.kDutyCycle);
    encoder.setPositionConversionFactor(360);
    wristNeo.getPIDController().setFeedbackDevice(encoder);
    wristNeo.getPIDController().setPositionPIDWrappingEnabled(true);
    wristNeo.getPIDController().setPositionPIDWrappingMinInput(0);
    wristNeo.getPIDController().setPositionPIDWrappingMaxInput(360);
    wristNeo.getPIDController().setP(.03);

    encoder.setInverted(true);
    encoder.setZeroOffset(0);

    wristNeo.setSoftLimit(SoftLimitDirection.kForward, WRIST_HIGH_LIM);
    wristNeo.setSoftLimit(SoftLimitDirection.kReverse, WRIST_LOW_LIM);

    wristNeo.enableSoftLimit(SoftLimitDirection.kForward, true);
    wristNeo.enableSoftLimit(SoftLimitDirection.kReverse, true);

    Util.setRevStatusRates(wristNeo, 10, 20, 65535, 65535, 65535, 20, 65535, 65535);

    SmartDashboard.putData(this);
    SmartDashboard.putData(enableBrakeMode());
    SmartDashboard.putData(enableCoastMode());
  }

  public Command setToTargetCmd(double target) {
    this.target = target;
    return runOnce(() -> wristNeo.getPIDController().setReference(target, ControlType.kPosition))
        .withName("setGoal[" + target + "]");
  }

  public Command incrementUp() {

    return runOnce(
        () -> {
          target++;
          wristNeo.getPIDController().setReference(target, ControlType.kPosition);
        });
  }

  public Command incrementDown() {
    return runOnce(
        () -> {
          target--;
          wristNeo.getPIDController().setReference(target, ControlType.kPosition);
        });
  }

  public void setToTarget(double target) {
    this.target = target;
    wristNeo.getPIDController().setReference(target, ControlType.kPosition);
  }

  public Command stow() {
    return setToTarget(stowPos).withName("Stow");
  }

  public Command enableCoastMode() {
    return Commands.runOnce(() -> wristNeo.setIdleMode(IdleMode.kCoast))
        .andThen(new PrintCommand("Coast Mode Set On Wrist"))
        .ignoringDisable(true)
        .withName("Enable Wrist Coast");
  }

  public Command enableBrakeMode() {
    return Commands.runOnce(() -> wristNeo.setIdleMode(IdleMode.kBrake))
        .andThen(new PrintCommand("Brake Mode Set On Wrist"))
        .ignoringDisable(true)
        .withName("Enable Wrist Brake");
  }

  public boolean isAtTarget() {
    return Math.abs(target - encoder.getPosition()) < 2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Wrist Enc Pos", encoder.getPosition());
  }
}
