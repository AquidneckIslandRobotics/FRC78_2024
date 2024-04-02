// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static com.revrobotics.CANSparkBase.*;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.classes.Util;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  private CANSparkMax wristNeo;
  private AbsoluteEncoder encoder;
  private double stowPos = 55;
  private double target = 0;

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.of(.5).per(Second), Volts.of(2), Seconds.of(10)),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) -> wristNeo.setVoltage(volts.in(Volts)),
              null,
              this,
              "wrist"));

  /** Creates a new Wrist. */
  public Wrist(int WRIST_ID, float WRIST_HIGH_LIM, float WRIST_LOW_LIM) {
    wristNeo = new CANSparkMax(WRIST_ID, MotorType.kBrushless);

    wristNeo.restoreFactoryDefaults();

    wristNeo.setIdleMode(IdleMode.kBrake);

    encoder = wristNeo.getAbsoluteEncoder(Type.kDutyCycle);
    encoder.setPositionConversionFactor(360);
    encoder.setVelocityConversionFactor(360.0 / 60.0);
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

    Util.setRevStatusRates(wristNeo, 10, 20, 32767, 32767, 32767, 20, 32767, 32767);

    SmartDashboard.putData(this);
    SmartDashboard.putData(enableBrakeMode());
    SmartDashboard.putData(enableCoastMode());
  }

  public Command setToTargetCmd(double target) {
    this.target = target;
    return runOnce(() -> wristNeo.getPIDController().setReference(target, ControlType.kPosition))
        .withName("setGoal[" + target + "]");
  }

  public void setToTarget(double target) {
    this.target = target;
    wristNeo.getPIDController().setReference(target, ControlType.kPosition);
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

  public Command stow() {
    return setToTargetCmd(stowPos).withName("Stow");
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

  private void configureMotorsBeforeSysId() {
    Util.setRevStatusRates(wristNeo, 5, 5, 32767, 32767, 32767, 5, 5, 32767);
  }

  private void configureMotorsAfterSysId() {
    Util.setRevStatusRates(wristNeo, 10, 20, 32767, 32767, 32767, 20, 32767, 32767);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return runOnce(this::configureMotorsBeforeSysId)
        .andThen(sysIdRoutine.quasistatic(direction))
        .finallyDo(this::configureMotorsAfterSysId);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return runOnce(this::configureMotorsBeforeSysId)
        .andThen(sysIdRoutine.dynamic(direction))
        .finallyDo(this::configureMotorsAfterSysId);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Wrist Enc Pos", encoder.getPosition());
  }
}
