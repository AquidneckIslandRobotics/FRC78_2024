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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  private CANSparkMax wristNeo;
  private AbsoluteEncoder encoder;
  private double stowPos = 50;

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
    encoder.setZeroOffset(0);

    wristNeo.setSoftLimit(SoftLimitDirection.kForward, WRIST_HIGH_LIM);
    wristNeo.setSoftLimit(SoftLimitDirection.kReverse, WRIST_LOW_LIM);

    wristNeo.enableSoftLimit(SoftLimitDirection.kForward, true);
    wristNeo.enableSoftLimit(SoftLimitDirection.kReverse, true);

    this.setDefaultCommand(setToTarget(WRIST_HIGH_LIM));
  }

  public Command setToTarget(double target) {
    return run(() -> wristNeo.getPIDController().setReference(target, ControlType.kPosition));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Wrist Enc Pos", encoder.getPosition());
  }
}
