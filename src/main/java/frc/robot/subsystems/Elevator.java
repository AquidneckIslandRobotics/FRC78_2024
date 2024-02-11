// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private CANSparkMax elevNeoMotor1;
  private CANSparkMax elevNeoMotor2;

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(2), Seconds.of(10)),
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

  private final double STOW_POSITION = 0.0;

  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
  private final SparkPIDController controller;

  private final TrapezoidProfile motionProfile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              InchesPerSecond.of(13), InchesPerSecond.per(Second).of(40)));

  private final TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
  private final TrapezoidProfile.State currentState = new TrapezoidProfile.State();

  public Elevator() {
    elevNeoMotor1 = new CANSparkMax(11, MotorType.kBrushless);
    elevNeoMotor2 = new CANSparkMax(12, MotorType.kBrushless);

    elevNeoMotor1.restoreFactoryDefaults();
    elevNeoMotor2.restoreFactoryDefaults();

    elevNeoMotor1.setIdleMode(IdleMode.kBrake);
    elevNeoMotor2.setIdleMode(IdleMode.kBrake);

    encoder = elevNeoMotor1.getEncoder();
    double inchesPerRevolution = 1.29 * Math.PI / 5 * 5;
    encoder.setPositionConversionFactor(inchesPerRevolution);
    // Inches per second
    encoder.setVelocityConversionFactor(inchesPerRevolution / 60);
    controller = elevNeoMotor1.getPIDController();
    controller.setP(180);
    elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kForward, false);
    elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kReverse, false);

    elevNeoMotor1.setInverted(false);
    elevNeoMotor2.follow(elevNeoMotor1, true);

    // Remaining status frames are unused
    elevNeoMotor1.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 65535);
    elevNeoMotor1.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 65535);
    elevNeoMotor1.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 65535);
    elevNeoMotor1.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 65535);

    // This motor will only follow, so no status frame are necessary
    elevNeoMotor2.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 65535);
    elevNeoMotor2.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 65535);
    elevNeoMotor2.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 65535);
    elevNeoMotor2.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 65535);
    elevNeoMotor2.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 65535);
    elevNeoMotor2.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 65535);
    elevNeoMotor2.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 65535);

    this.setDefaultCommand(run(this::moveElevator));
  }

  public boolean elevatorIsStowed() {
    return zeroed && encoder.getPosition() <= .5;
  }

  private Command lowerElevatorUntilLimitReached() {
    return run(() -> elevNeoMotor1.set(-.1))
        .until(() -> !reverseLimitSwitch.get())
        .finallyDo(() -> elevNeoMotor1.set(0));
  }

  private void configureLeaderBeforeSysId() {
    // Applied Output
    elevNeoMotor1.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10);
    // Motor Velocity and Voltage
    elevNeoMotor1.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 10);
    // Motor Position
    elevNeoMotor1.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 10);
  }

  private void configureLeaderAfterSysId() {
    // Applied Output
    elevNeoMotor1.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 65535);
    // Motor Velocity and Voltage
    elevNeoMotor1.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 10);
    // Motor Position
    elevNeoMotor1.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 10);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return runOnce(this::configureLeaderBeforeSysId)
        .andThen(sysIdRoutine.quasistatic(direction))
        .finallyDo(
            () -> {
              configureLeaderAfterSysId();
              holdPosition();
            });
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return runOnce(this::configureLeaderBeforeSysId)
        .andThen(sysIdRoutine.dynamic(direction))
        .finallyDo(
            () -> {
              configureLeaderAfterSysId();
              holdPosition();
            });
  }

  private Command configureMotorsAfterZeroing() {
    return runOnce(
        () -> {
          encoder.setPosition(0);
          goal.position = 0;
          goal.velocity = 0;
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
        .andThen(stow())
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
    Logger.recordOutput("Elevator/AppliedVoltage", appliedOutput);
    Logger.recordOutput("Elevator/Goal", goal.position);
  }

  private void setGoal(double goal) {
    this.goal.position = Units.inchesToMeters(goal);
  }

  /**
   * Set the elevator goal height
   *
   * @param goal Goal height for the elevator in inches.
   */
  public Command setGoalCommand(double goal) {
    return runOnce(() -> setGoal(goal));
  }

  private void holdPosition() {
    setGoal(encoder.getPosition());
  }

  public Command stow() {
    return setGoalCommand(STOW_POSITION);
  }

  /**
   * This method actually moves the wrist. The default command will move the wrist to the current
   * goal
   */
  private void moveElevator() {
    if (!zeroed) return;
    currentState.position = Units.inchesToMeters(encoder.getPosition());
    currentState.velocity = InchesPerSecond.of(encoder.getVelocity()).in(MetersPerSecond);
    TrapezoidProfile.State nextSetPoint = motionProfile.calculate(kDt, currentState, goal);
    double ff =
        feedforward.calculate(MetersPerSecond.of(nextSetPoint.velocity).in(InchesPerSecond));
    controller.setReference(nextSetPoint.position, CANSparkBase.ControlType.kPosition, 0, ff);
  }
}
