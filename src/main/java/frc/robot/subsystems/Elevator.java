// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.classes.Util;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private CANSparkMax elevNeoMotor1;
  private CANSparkMax elevNeoMotor2;

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(1).per(Second),
              Volts.of(2),
              Seconds.of(10),
              (state) -> Logger.recordOutput("SysIdTestState-Elevator", state.toString())),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) -> elevNeoMotor1.setVoltage(volts.in(Volts)),
              null,
              this,
              "elevator"));

  private DigitalInput reverseLimitSwitch = new DigitalInput(0);
  private boolean zeroed = false;

  public boolean hasNotBeenZeroed() {
    return !zeroed;
  }

  private final RelativeEncoder encoder;

  private final double kS = 0.070936;
  private final double kV = 0.79005;
  private final double kA = 0.086892;
  private final double kG = 0.088056;
  // Command loop runs at 50Hz, 20ms period
  private final double kDt = 0.02;
  private double appliedOutput = 0;

  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
  private final ProfiledPIDController profiledPid =
      new ProfiledPIDController(
          180,
          0,
          0,
          new TrapezoidProfile.Constraints(
              InchesPerSecond.of(13), InchesPerSecond.per(Second).of(40)),
          kDt);

  private final double STOW_POSITION = 0;

  public Elevator() {
    elevNeoMotor1 = new CANSparkMax(11, MotorType.kBrushless);
    elevNeoMotor2 = new CANSparkMax(12, MotorType.kBrushless);

    elevNeoMotor1.restoreFactoryDefaults();
    elevNeoMotor2.restoreFactoryDefaults();

    elevNeoMotor1.setIdleMode(IdleMode.kBrake);
    elevNeoMotor2.setIdleMode(IdleMode.kBrake);

    encoder = elevNeoMotor1.getEncoder();
    double inchesPerRevolution = 1.29 * Math.PI / (5 * 5);
    encoder.setPositionConversionFactor(inchesPerRevolution);
    // Inches per second
    encoder.setVelocityConversionFactor(inchesPerRevolution / 60);
    elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kForward, false);
    elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kReverse, false);

    elevNeoMotor1.setInverted(false);
    elevNeoMotor2.follow(elevNeoMotor1, true);

    Util.setRevStatusRates(elevNeoMotor1, 5, 20, 20, 65535, 65535, 65535, 65535, 65535);
    Util.setRevStatusRates(elevNeoMotor2, 500, 65535, 65535, 65535, 65535, 65535, 65535, 65535);

    this.setDefaultCommand(run(this::moveElevator));
  }

  public boolean elevatorIsStowed() {
    return zeroed && encoder.getPosition() <= .5;
  }

  private Command lowerElevatorUntilLimitReached() {
    return run(() -> elevNeoMotor1.set(-.1))
        .until(() -> !reverseLimitSwitch.get())
        .andThen(runOnce(() -> elevNeoMotor1.set(0)));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  private Command configureMotorsAfterZeroing() {
    return runOnce(
        () -> {
          encoder.setPosition(0);
          profiledPid.setGoal(0);
          elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kForward, true);
          elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kReverse, true);
          elevNeoMotor1.setSoftLimit(SoftLimitDirection.kForward, 15);
          elevNeoMotor1.setSoftLimit(SoftLimitDirection.kReverse, 0);
          zeroed = true;
        });
  }

  public Command zeroElevator() {
    return lowerElevatorUntilLimitReached()
        .andThen(configureMotorsAfterZeroing())
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  public void periodic() {
    Logger.recordOutput("Elevator/limit pressed", !reverseLimitSwitch.get());
    Logger.recordOutput("Elevator/zeroed", zeroed);
    Logger.recordOutput("Elevator/position", encoder.getPosition());
    Logger.recordOutput(
        "Elevator/reverse limit reached", elevNeoMotor1.getFault(FaultID.kSoftLimitRev));
    Logger.recordOutput(
        "Elevator/forward limit reached", elevNeoMotor1.getFault(FaultID.kSoftLimitFwd));
    Logger.recordOutput("Elevator/PIDoutput", profiledPid.getPositionError());
    Logger.recordOutput("Elevator/Profile Velocity", profiledPid.getSetpoint().velocity);
    Logger.recordOutput("Elevator/AppliedVoltage", appliedOutput);
    Logger.recordOutput("Elevator/Goal", profiledPid.getSetpoint().position);
  }

  /**
   * Set the elevator goal height
   *
   * @param goal Goal height for the elevator in inches.
   */
  public Command setGoal(double goal) {
    return runOnce(() -> profiledPid.setGoal(Units.inchesToMeters(goal)));
  }

  public void holdPosition() {
    profiledPid.setGoal(encoder.getPosition());
  }

  public Command stow() {
    return setGoal(STOW_POSITION);
  }

  /**
   * This method actually moves the wrist. The default command will move the wrist to the current
   * goal
   */
  private void moveElevator() {
    if (!zeroed) return;
    double ff =
        feedforward.calculate(
            MetersPerSecond.of(profiledPid.getSetpoint().velocity).in(InchesPerSecond));
    double feedback = profiledPid.calculate(Units.inchesToMeters(encoder.getPosition()));
    elevNeoMotor1.setVoltage(feedback + ff);
  }
}
