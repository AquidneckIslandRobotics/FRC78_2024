// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {

  private CANSparkMax wristNeo;
  private AbsoluteEncoder encoder;

  private final double kS = 0;
  private final double kV = 2.1;
  private final double kA = 0;
  private final double kG = 0.27;

  private final double kDt = 0.02;

  private final Measure<Velocity<Distance>> manualSpeed = InchesPerSecond.of(1);

  private final ArmFeedforward feedforwardWrist = new ArmFeedforward(kS, kV, kA, kG);
  private final ProfiledPIDController profilePIDWrist =
      new ProfiledPIDController(
          0.03,
          0,
          0,
          new TrapezoidProfile.Constraints(
              InchesPerSecond.of(8), InchesPerSecond.per(Seconds).of(6)));

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

    this.setDefaultCommand(setToTarget(139));
  }

  public Command setToTarget(double target) {
    return runOnce(() -> profilePIDWrist.setGoal(target))
        .andThen(
            run(
                () -> {
                  wristNeo.setVoltage(
                      profilePIDWrist.calculate(Units.metersToInches(encoder.getPosition()))
                          + feedforwardWrist.calculate(
                              encoder.getPosition(),
                              MetersPerSecond.of(profilePIDWrist.getSetpoint().velocity)
                                  .in(InchesPerSecond)));
                }));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("currentPosWrist", encoder.getPosition());
  }
}
