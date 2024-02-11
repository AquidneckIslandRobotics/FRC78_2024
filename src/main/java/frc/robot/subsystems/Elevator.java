// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private CANSparkMax elevNeoMotor1;
  private CANSparkMax elevNeoMotor2;

  private DigitalInput reverseLimitSwitch = new DigitalInput(0);
  private boolean zeroed = false;

  private RelativeEncoder encoder;

  public boolean hasNotBeenZeroed() {
    return !zeroed;
  }

  /** Creates a new Elevator. */
  private final double kS = 0.070936;

  private final double kV = 0.79005;
  private final double kA = 0.086892;
  private final double kG = 0.088056;
  // Command loop runs at 50Hz, 20ms period
  private final double kDt = 0.02;

  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

  private final TrapezoidProfile motionProfile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              InchesPerSecond.of(3), InchesPerSecond.per(Second).of(3)));

  private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(0, 0);

  public Elevator() {
    elevNeoMotor1 = new CANSparkMax(11, MotorType.kBrushless);
    elevNeoMotor1.restoreFactoryDefaults();
    elevNeoMotor1.setIdleMode(IdleMode.kBrake);
    encoder = elevNeoMotor1.getEncoder();
    elevNeoMotor1.getEncoder().setPositionConversionFactor(1.29 * Math.PI / (5 * 5));
    elevNeoMotor1.getPIDController().setP(5.7255);
    elevNeoMotor1.getPIDController().setD(0.26639);
    elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kForward, false);
    elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kReverse, false);

    elevNeoMotor1.setInverted(false);

    elevNeoMotor2 = new CANSparkMax(12, MotorType.kBrushless);
    elevNeoMotor2.restoreFactoryDefaults();
    elevNeoMotor2.setIdleMode(IdleMode.kBrake);
    elevNeoMotor2.follow(elevNeoMotor1, true);

    this.setDefaultCommand(setToTarget(0));
  }

  private Command lowerElevatorUntilLimitReached() {
    return run(() -> elevNeoMotor1.set(-.1)).until(reverseLimitSwitch::get);
  }

  private Command configureMotorsAfterZeroing() {
    return runOnce(
        () -> {
          encoder.setPosition(0);
          goal.position = 0;
          goal.velocity = 0;
          elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kForward, true);
          elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kReverse, true);
          elevNeoMotor1.setSoftLimit(SoftLimitDirection.kForward, 16.5f);
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
    return run(() -> elevNeoMotor1.set(.1));
  }

  /** Manually move elevator down by gradually moving the setpoint. */
  public Command moveElevatorDown() {
    return run(() -> elevNeoMotor1.set(-.1));
  }

  public void periodic() {
    SmartDashboard.putBoolean("limit pressed", reverseLimitSwitch.get());
    SmartDashboard.putBoolean("zeroed", zeroed);
    SmartDashboard.putNumber("position", encoder.getPosition());
    SmartDashboard.putBoolean(
        "reverse limit reached", elevNeoMotor1.getFault(FaultID.kSoftLimitRev));
    SmartDashboard.putBoolean(
        "forward limit reached", elevNeoMotor1.getFault(FaultID.kSoftLimitFwd));
    SmartDashboard.putNumber("Elevator Profile Velocity", setpoint.velocity);
  }

  /** Moves elevator to target as long as elevator is zeroed */
  public Command setToTarget(double target) {
    return runOnce(
            () -> {
              goal.position = target;
              setpoint = new TrapezoidProfile.State(encoder.getPosition(), encoder.getVelocity());
            })
        .andThen(
            run(
                () -> {
                  if (!zeroed) return;

                  setpoint = motionProfile.calculate(kDt, setpoint, goal);
                  elevNeoMotor1
                      .getPIDController()
                      .setReference(
                          setpoint.position,
                          CANSparkBase.ControlType.kPosition,
                          0,
                          feedforward.calculate(
                              MetersPerSecond.of(setpoint.velocity).in(InchesPerSecond)));
                }));
  }
}
