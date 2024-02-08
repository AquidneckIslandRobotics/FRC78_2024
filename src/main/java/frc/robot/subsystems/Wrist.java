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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {

  private CANSparkMax wristNeo;
  private AbsoluteEncoder encoder;

  private double highLim;
  private double lowLim;

  private double stow;

  /** Creates a new Wrist. */
  public Wrist(int WRIST_ID, float WRIST_HIGH_LIM, float WRIST_LOW_LIM) {
    wristNeo = new CANSparkMax(WRIST_ID, MotorType.kBrushless);

    wristNeo.restoreFactoryDefaults();

    wristNeo.setIdleMode(IdleMode.kBrake);

    encoder = wristNeo.getAbsoluteEncoder(Type.kDutyCycle);
    encoder.setPositionConversionFactor(360);
    wristNeo.getPIDController().setFeedbackDevice(encoder);
    wristNeo.getPIDController().setP(.03);

    encoder.setInverted(true);
    encoder.setZeroOffset(340);

    wristNeo.setSoftLimit(SoftLimitDirection.kForward, WRIST_HIGH_LIM);
    wristNeo.setSoftLimit(SoftLimitDirection.kReverse, WRIST_LOW_LIM);

    wristNeo.enableSoftLimit(SoftLimitDirection.kForward, false);
    wristNeo.enableSoftLimit(SoftLimitDirection.kReverse, false);

    highLim = WRIST_HIGH_LIM;
    lowLim = WRIST_LOW_LIM;

    stow = 130;
  }

  public void SetSpeed(double speed) {
    wristNeo.set(speed);
  }

  public boolean atHighLimit() {
    double wristPosition = encoder.getPosition();
    if (wristPosition >= highLim) {
      return true;
    } else {
      return false;
    }
  }

  public boolean atLowLimit() {
    double wristPosition = encoder.getPosition();
    if (wristPosition <= lowLim) {
      return true;
    } else {
      return false;
    }
  }

  public void wristToPosition(double target) {
    double currentWristPos = encoder.getPosition();
    if (currentWristPos > target) {
      wristNeo.set(-0.5);
    } else if (currentWristPos < target) {
      wristNeo.set(0.5);
    } else {
      wristNeo.set(0);
    }
  }

  public Command moveWristUp() {
    return this.startEnd(() -> wristNeo.set(.3), () -> wristNeo.set(0));
  }

  public Command moveWristDown() {
    return this.startEnd(() -> wristNeo.set(-.3), () -> wristNeo.set(0));
  }

  public Command setToTarget(double target) {
    return runOnce(() -> wristNeo.getPIDController().setReference(target, ControlType.kPosition));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("currentPosWrist", encoder.getPosition());
  }
}
