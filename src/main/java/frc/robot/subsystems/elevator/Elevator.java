// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.elevator.Constants.constraints;
import static frc.robot.subsystems.elevator.Constants.drumRadius;
import static frc.robot.subsystems.elevator.Constants.followerMotorId;
import static frc.robot.subsystems.elevator.Constants.kA;
import static frc.robot.subsystems.elevator.Constants.kCarriageMass;
import static frc.robot.subsystems.elevator.Constants.kD;
import static frc.robot.subsystems.elevator.Constants.kDt;
import static frc.robot.subsystems.elevator.Constants.kFollowerMotorInverted;
import static frc.robot.subsystems.elevator.Constants.kFollowerMotorStatusRates;
import static frc.robot.subsystems.elevator.Constants.kG;
import static frc.robot.subsystems.elevator.Constants.kGearRatio;
import static frc.robot.subsystems.elevator.Constants.kI;
import static frc.robot.subsystems.elevator.Constants.kLeaderMotorInverted;
import static frc.robot.subsystems.elevator.Constants.kMaxHeight;
import static frc.robot.subsystems.elevator.Constants.kMinHeight;
import static frc.robot.subsystems.elevator.Constants.kP;
import static frc.robot.subsystems.elevator.Constants.kPositionConversionFactor;
import static frc.robot.subsystems.elevator.Constants.kS;
import static frc.robot.subsystems.elevator.Constants.kSimStartingHeight;
import static frc.robot.subsystems.elevator.Constants.kSoftLimitForward;
import static frc.robot.subsystems.elevator.Constants.kTolerance;
import static frc.robot.subsystems.elevator.Constants.kV;
import static frc.robot.subsystems.elevator.Constants.leaderMotorId;
import static frc.robot.subsystems.elevator.Constants.leaderMotorStatusRates;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.Util;
import frc.robot.test.Robot;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private CANSparkMax leaderMotor;
  private CANSparkMax followerMotor;

  private SparkLimitSwitch reverseLimitSwitch;
  private boolean zeroed = false;

  public boolean hasNotBeenZeroed() {
    return !zeroed;
  }

  private RelativeEncoder encoder;

  private double appliedOutput = 0;

  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
  private final ProfiledPIDController profiledPid =
      new ProfiledPIDController(kP, kI, kD, constraints, kDt);

  private final Mechanism2d mech2d = new Mechanism2d(.5, 1);
  private final MechanismRoot2d root2d = mech2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d elevatorMech = root2d.append(
      new MechanismLigament2d("Elevator", 0, 90));

  public Elevator() {
    leaderMotor = new CANSparkMax(leaderMotorId, MotorType.kBrushless);
    followerMotor = new CANSparkMax(followerMotorId, MotorType.kBrushless);

    leaderMotor.restoreFactoryDefaults();
    followerMotor.restoreFactoryDefaults();

    leaderMotor.setIdleMode(IdleMode.kBrake);
    followerMotor.setIdleMode(IdleMode.kBrake);

    encoder = leaderMotor.getEncoder();
    encoder.setPositionConversionFactor(kPositionConversionFactor);
    leaderMotor.getPIDController().setFeedbackDevice(encoder);
    leaderMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    leaderMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    profiledPid.setTolerance(kTolerance);

    reverseLimitSwitch = leaderMotor.getReverseLimitSwitch(Type.kNormallyOpen);

    leaderMotor.setInverted(kLeaderMotorInverted);
    followerMotor.follow(leaderMotor, kFollowerMotorInverted);

    Util.setRevStatusRates(leaderMotor, leaderMotorStatusRates);
    Util.setRevStatusRates(followerMotor, kFollowerMotorStatusRates);

    this.setDefaultCommand(setToTarget(0));
    SmartDashboard.putData(enableCoastMode());
    SmartDashboard.putData(enableBrakeMode());
    SmartDashboard.putData("Elevator Profile", profiledPid);
    SmartDashboard.putData(this);

    if (Robot.isSimulation()) {

      REVPhysicsSim.getInstance().addSparkMax(leaderMotor, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(followerMotor, DCMotor.getNEO(1));

      SmartDashboard.putData("Elevator Sim", mech2d);
    }
  }

  public boolean elevatorIsStowed() {
    return zeroed && encoder.getPosition() <= .5;
  }

  public boolean elevIsAtPos() {
    return profiledPid.atGoal();
  }

  private Command lowerElevatorUntilLimitReached() {
    return run(() -> leaderMotor.set(-.1)).until(() -> reverseLimitSwitch.isPressed());
  }

  private Command configureMotorsAfterZeroing() {
    return runOnce(
        () -> {
          encoder.setPosition(0);
          profiledPid.setGoal(0);
          leaderMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
          leaderMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
          leaderMotor.setSoftLimit(SoftLimitDirection.kForward, kSoftLimitForward);
          leaderMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
          zeroed = true;
          this.setDefaultCommand(setToTarget(0));
        })
        .withName("Configure Motors After Zeroing");
  }

  public Command zeroElevator() {
    if (Robot.isSimulation()) {
      return configureMotorsAfterZeroing();
    } else {
      return lowerElevatorUntilLimitReached()
          .andThen(configureMotorsAfterZeroing())
          .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
          .withName("Zero Elevator");
    }
  }

  public Command enableCoastMode() {
    return Commands.runOnce(
                       () -> {
                         leaderMotor.setIdleMode(IdleMode.kCoast);
                         followerMotor.setIdleMode(IdleMode.kCoast);
                       })
                   .andThen(new PrintCommand("Coast Mode Set On Elevator"))
                   .ignoringDisable(true)
                   .withName("Enable Elevator Coast");
  }

  public Command enableBrakeMode() {
    return Commands.runOnce(
                       () -> {
                         leaderMotor.setIdleMode(IdleMode.kBrake);
                         followerMotor.setIdleMode(IdleMode.kBrake);
                       })
                   .andThen(new PrintCommand("Brake Mode Set On Elevator"))
                   .ignoringDisable(true)
                   .withName("Enable Elevator Brake");
  }

  public void periodic() {
    Logger.recordOutput("Elevator/limit pressed", reverseLimitSwitch.isPressed());
    Logger.recordOutput("Elevator/zeroed", zeroed);
    Logger.recordOutput("Elevator/position", encoder.getPosition());
    Logger.recordOutput(
        "Elevator/reverse limit reached", leaderMotor.getFault(FaultID.kSoftLimitRev));
    Logger.recordOutput(
        "Elevator/forward limit reached", leaderMotor.getFault(FaultID.kSoftLimitFwd));
    Logger.recordOutput("Elevator/PIDoutput", profiledPid.getPositionError());
    Logger.recordOutput("Elevator/Profile Velocity", profiledPid.getSetpoint().velocity);
    Logger.recordOutput("Elevator/AppliedVoltage", appliedOutput);
    Logger.recordOutput("Elevator/Goal", profiledPid.getSetpoint().position);
  }

  public void simulationPeriodic() {
    elevatorMech.setLength(encoder.getPosition());
  }


  public double getElevatorPos() {
    return encoder.getPosition();
  }

  /**
   * Moves elevator to target as long as elevator is zeroed
   */
  public Command setToTarget(double target) {
    return new FunctionalCommand(
        () -> profiledPid.setGoal(target),
        () -> {
          if (!zeroed) {
            return;
          }
          appliedOutput =
              profiledPid.calculate(encoder.getPosition())
              + feedforward.calculate(profiledPid.getSetpoint().velocity);
          leaderMotor.setVoltage(appliedOutput);
        }, (interrupted) -> {
    },
        () -> false,
        this
    )
        .withName("setTo[" + target + "]");
  }
}
