// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.classes.Util;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class Wrist extends SubsystemBase {

  private CANSparkMax wristNeo;
  private AbsoluteEncoder encoder;
  private double stowPos = 55;

  private final ArmFeedforward feedforward = new ArmFeedforward(0, .31, 0, 0);

  private final ProfiledPIDController profiledPid =
      new ProfiledPIDController(0.03, 0, 0, new TrapezoidProfile.Constraints(50, 50));


  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
          new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(3), Seconds.of(4), (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
          new SysIdRoutine.Mechanism(voltageMeasure -> wristNeo.setVoltage(voltageMeasure.in(Volts)), null, this, "wrist"));
  )

  /** Creates a new Wrist. */
  public Wrist(int WRIST_ID, float WRIST_HIGH_LIM, float WRIST_LOW_LIM) {
    wristNeo = new CANSparkMax(WRIST_ID, MotorType.kBrushless);

    wristNeo.restoreFactoryDefaults();

    wristNeo.setIdleMode(IdleMode.kBrake);

    encoder = wristNeo.getAbsoluteEncoder(Type.kDutyCycle);
    encoder.setPositionConversionFactor(360);
    profiledPid.enableContinuousInput(0, 360);

    encoder.setInverted(true);
    encoder.setZeroOffset(0);

    wristNeo.setSoftLimit(SoftLimitDirection.kForward, WRIST_HIGH_LIM);
    wristNeo.setSoftLimit(SoftLimitDirection.kReverse, WRIST_LOW_LIM);

    wristNeo.enableSoftLimit(SoftLimitDirection.kForward, true);
    wristNeo.enableSoftLimit(SoftLimitDirection.kReverse, true);

    profiledPid.setTolerance(2);
    profiledPid.setGoal(new TrapezoidProfile.State(WRIST_HIGH_LIM, 0));

    Util.setRevStatusRates(wristNeo, 10, 20, 32767, 32767, 32767, 20, 32767, 32767);

    SmartDashboard.putData(this);
    SmartDashboard.putData(enableBrakeMode());
    SmartDashboard.putData(enableCoastMode());

    setDefaultCommand(moveWrist());
  }

  private Command moveWrist() {
    return run(() -> {
          double ff =
              feedforward.calculate(Units.degreesToRadians(encoder.getPosition() - 180 - 42), 0);

          double fb = profiledPid.calculate(encoder.getPosition());
          wristNeo.setVoltage(ff + fb);
        })
        .withName("MoveWrist");
  }

  public Command setToTargetCmd(double target) {
    return Commands.runOnce(() -> profiledPid.setGoal(target)).withName("setGoal[" + target + "]");
  }

  public void setToTarget(double target) {
    profiledPid.setGoal(target);
  }

  public Command stow() {
    return setToTargetCmd(stowPos).withName("Stow").withName("Stow");
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
    return profiledPid.atGoal();
  }

  public Command sysIdRoutine() {
    return Commands.sequence(
            runOnce(() -> Util.setRevStatusRates(wristNeo, 10, 20, 32767, 32767, 32767, 20, 20, 32767)),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(() -> encoder.getPosition() < 5),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(() -> encoder.getPosition() < 5),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
            runOnce(() -> Util.setRevStatusRates(wristNeo, 10, 20, 32767, 32767, 32767, 20, 32767, 32767))
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Wrist target", profiledPid.getGoal().position);
    Logger.recordOutput(
        "Wrist ff",
        feedforward.calculate(Units.degreesToRadians(encoder.getPosition() - 180 - 42), 0));
    Logger.recordOutput("Wrist Enc Pos", encoder.getPosition());
  }
}
