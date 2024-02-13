// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private CANSparkMax elevNeoMotor1;
  private CANSparkMax elevNeoMotor2;

  private DigitalInput reverseLimitSwitch = new DigitalInput(0);
  private boolean zeroed = false;

  public boolean hasNotBeenZeroed() {
    return !zeroed;
  }

  private RelativeEncoder encoder;

  private final double kS = 0.035369;
  private final double kV = 0.52479;
  private final double kA = 0.029988;
  private final double kG = 0.029988;
  // Command loop runs at 50Hz, 20ms period
  private final double kDt = 0.02;

  private final Measure<Velocity<Distance>> manualSpeed = InchesPerSecond.of(1);

  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
  private final ProfiledPIDController profiledPid =
      new ProfiledPIDController(
          5.6495,
          0,
          0.15652,
          new TrapezoidProfile.Constraints(
              InchesPerSecond.of(8), InchesPerSecond.per(Second).of(6)),
          kDt);

  public Elevator() {
    elevNeoMotor1 = new CANSparkMax(11, MotorType.kBrushless);
    elevNeoMotor2 = new CANSparkMax(12, MotorType.kBrushless);

    elevNeoMotor1.restoreFactoryDefaults();
    elevNeoMotor2.restoreFactoryDefaults();

    elevNeoMotor1.setIdleMode(IdleMode.kBrake);
    elevNeoMotor2.setIdleMode(IdleMode.kBrake);

    encoder = elevNeoMotor1.getEncoder();
    encoder.setPositionConversionFactor((1.29 * Math.PI) / 25);
    elevNeoMotor1.getPIDController().setFeedbackDevice(encoder);
    elevNeoMotor1.getPIDController().setP(.077);
    elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kForward, false);
    elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kReverse, false);

    elevNeoMotor1.setInverted(false);
    elevNeoMotor2.follow(elevNeoMotor1, true);

    this.setDefaultCommand(setToTarget(0));
  }

  private Command lowerElevatorUntilLimitReached() {
    return run(() -> elevNeoMotor1.set(-.1)).until(() -> !reverseLimitSwitch.get());
  }

  private Command configureMotorsAfterZeroing() {
    return runOnce(
        () -> {
          encoder.setPosition(0);
          profiledPid.setGoal(0);
          elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kForward, true);
          elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kReverse, true);
          elevNeoMotor1.setSoftLimit(SoftLimitDirection.kForward, 14);
          elevNeoMotor1.setSoftLimit(SoftLimitDirection.kReverse, 0);
          zeroed = true;
        });
  }

  public Command zeroElevator() {
    return lowerElevatorUntilLimitReached()
        .andThen(configureMotorsAfterZeroing())
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  /** Manually move elevator up by gradually moving the setpoint. */
  public Command moveElevatorUp() {
    return Commands.runOnce(
        () ->
            profiledPid.setGoal(
                encoder.getPosition() + manualSpeed.times(kDt).in(InchesPerSecond)));
  }

  /** Manually move elevator down by gradually moving the setpoint. */
  public Command moveElevatorDown() {
    return Commands.runOnce(
        () ->
            profiledPid.setGoal(
                encoder.getPosition() - manualSpeed.times(kDt).in(InchesPerSecond)));
  }

  public void periodic() {
    SmartDashboard.putBoolean("limit pressed", !reverseLimitSwitch.get());
    SmartDashboard.putBoolean("zeroed", zeroed);
    SmartDashboard.putNumber("position", encoder.getPosition());
    SmartDashboard.putBoolean(
        "reverse limit reached", elevNeoMotor1.getFault(FaultID.kSoftLimitRev));
    SmartDashboard.putBoolean(
        "forward limit reached", elevNeoMotor1.getFault(FaultID.kSoftLimitFwd));
    SmartDashboard.putNumber("Elevator Profile Velocity", profiledPid.getSetpoint().velocity);
  }

  /** Moves elevator to target as long as elevator is zeroed */
  public Command setToTarget(double target) {
    return runOnce(() -> profiledPid.setGoal(target))
        .andThen(
            run(
                () -> {
                  if (!zeroed) return;
                  elevNeoMotor1.setVoltage(
                      profiledPid.calculate(Units.metersToInches(encoder.getPosition()))
                          + feedforward.calculate(
                              MetersPerSecond.of(profiledPid.getSetpoint().velocity)
                                  .in(InchesPerSecond)));
                }));
  }
}
