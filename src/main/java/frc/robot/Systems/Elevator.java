// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Elevator extends SubsystemBase {
  private final CANSparkMax elevatorMotor;

  /* The Rev Throughbore Encoder attached to the top of the elevator mechanism.
   * Connected to the data port via alternate encoder board */
  RelativeEncoder encoder;

  /* These values were taken from SysId */
  ElevatorFeedforward feedforwardController =
      new ElevatorFeedforward(1.0824, 0.69788, 10.813, 0.67364);
  TrapezoidProfile.Constraints motionProfileConstraints = new TrapezoidProfile.Constraints(1, 40);
  ProfiledPIDController feedbackController =
      new ProfiledPIDController(5.7441, 0, 1.6931, motionProfileConstraints, 0.02);

  // Creates a SysIdRoutine
  SysIdRoutine routine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this, "Elevator"));

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> voltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> velocity = mutable(MetersPerSecond.of(0));

  private void voltageDrive(Measure<Voltage> input) {
    elevatorMotor.setVoltage(input.in(Volts));
  }

  private void logMotors(SysIdRoutineLog log) {
    log.motor("motor")
        .voltage(
            voltage.mut_replace(
                elevatorMotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
        .linearPosition(distance.mut_replace(encoder.getPosition(), Meters))
        .linearVelocity(velocity.mut_replace(encoder.getVelocity(), MetersPerSecond));
  }

  /** Creates a new Elevator. */
  public Elevator(CANSparkMax motor) {
    elevatorMotor = motor;
    encoder = elevatorMotor.getAlternateEncoder(8192);
  }

  @Override
  public void periodic() {
    double fb = feedbackController.calculate(encoder.getPosition());
    double ff = feedforwardController.calculate(feedbackController.getSetpoint().velocity);

    // Only enable this after limit switches are installed and tested
    // elevatorMotor.set(fb + ff);

    SmartDashboard.putNumber("Goal", feedbackController.getGoal().position);
    SmartDashboard.putNumber("Setpoint", feedbackController.getSetpoint().position);
    SmartDashboard.putNumber("Motor Output", elevatorMotor.get());
    SmartDashboard.putNumber("Feedback", fb);
    SmartDashboard.putNumber("Feedforward", ff);
  }

  public Command moveToTop() {
    return this.runOnce(() -> feedbackController.setGoal(1.5));
  }

  public Command moveToMiddle() {
    return this.runOnce(() -> feedbackController.setGoal(0.50));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }
}
