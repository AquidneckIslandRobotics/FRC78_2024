// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static com.revrobotics.CANSparkBase.*;
import static com.revrobotics.CANSparkBase.ControlType.*;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.classes.Util;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  private CANSparkMax wristNeo;
  private AbsoluteEncoder encoder;

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(.5).per(Second),
              Volts.of(2),
              Seconds.of(10),
              (state) -> Logger.recordOutput("SysIdTestState-Wrist", state.toString())),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) -> wristNeo.setVoltage(volts.in(Volts)),
              null,
              this,
              "wrist"));

  private double STOW_ANGLE;

  /** The goal position of the wrist, in degrees */
  private double goal = STOW_ANGLE;

  private SparkPIDController controller;

  // TODO tune the wrist
  private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);

  /**
   * The center of mass for the wrist does not perfectly line up with 0º. We need to add this offset
   * so that the current encoder position lines up with the phase of gravity
   */
  private double kGPhaseOffset = 0;

  /** Creates a new Wrist. */
  public Wrist(int WRIST_ID, float WRIST_HIGH_LIM, float WRIST_LOW_LIM) {
    this.STOW_ANGLE = WRIST_HIGH_LIM;
    wristNeo = new CANSparkMax(WRIST_ID, MotorType.kBrushless);

    wristNeo.restoreFactoryDefaults();

    wristNeo.setIdleMode(IdleMode.kBrake);

    encoder = wristNeo.getAbsoluteEncoder(Type.kDutyCycle);
    encoder.setPositionConversionFactor(360);
    encoder.setVelocityConversionFactor(360.0 / 60.0);
    this.controller = wristNeo.getPIDController();
    controller.setP(.03);

    encoder.setInverted(true);
    encoder.setZeroOffset(0);

    wristNeo.setSoftLimit(SoftLimitDirection.kForward, WRIST_HIGH_LIM);
    wristNeo.setSoftLimit(SoftLimitDirection.kReverse, WRIST_LOW_LIM);

    wristNeo.enableSoftLimit(SoftLimitDirection.kForward, true);
    wristNeo.enableSoftLimit(SoftLimitDirection.kReverse, true);

    Util.setRevStatusRates(wristNeo, 10, 20, 65535, 65535, 65535, 20, 65535, 65535);

    setDefaultCommand(run(this::moveWrist));
  }

  public Command incrementAngle() {
    return runOnce(() -> goal += 1);
  }

  public Command decrementAngle() {
    return runOnce(() -> goal -= 1);
  }

  public boolean isAtTarget() {
    return Math.abs(goal - encoder.getPosition()) < 2;
  }

  private void configureMotorsBeforeSysId() {
    Util.setRevStatusRates(wristNeo, 5, 5, 65535, 65535, 65535, 5, 5, 65535);
  }

  private void configureMotorsAfterSysId() {
    Util.setRevStatusRates(wristNeo, 10, 20, 65535, 65535, 65535, 20, 65535, 65535);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return runOnce(this::configureMotorsBeforeSysId)
        .andThen(sysIdRoutine.quasistatic(direction))
        .andThen(holdPosition())
        .andThen(this::configureMotorsAfterSysId);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return runOnce(this::configureMotorsBeforeSysId)
        .andThen(sysIdRoutine.dynamic(direction))
        .andThen(holdPosition())
        .andThen(this::configureMotorsAfterSysId);
  }

  /**
   * Moves the wrist to a position when the command is scheduled, and resets to stow when the
   * command ends.
   *
   * <p>This will work with both whileTrue and toggleOnTrue for joystick bindings
   *
   * @param goal Goal angle for the wrist in degrees.
   */
  public Command setGoal(double goal) {
    return runOnce(() -> this.goal = goal);
  }

  private Command holdPosition() {
    return runOnce(() -> this.goal = encoder.getPosition());
  }

  public Command stow() {
    return setGoal(STOW_ANGLE);
  }

  /**
   * This method actually moves the wrist. The default command will move the wrist to the current
   * goal
   */
  private void moveWrist() {
    double ff = feedforward.calculate(encoder.getPosition() + kGPhaseOffset, 0);
    controller.setReference(goal, kPosition, 0, ff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Wrist Enc Pos", encoder.getPosition());
    SmartDashboard.putNumber("wrist/goal", goal);
  }
}
