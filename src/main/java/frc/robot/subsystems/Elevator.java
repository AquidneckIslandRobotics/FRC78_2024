// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private CANSparkMax elevNeoMotor1;
  private CANSparkMax elevNeoMotor2;

  /** Creates a new Elevator. */
  public Elevator() {
    elevNeoMotor1 = new CANSparkMax(11, MotorType.kBrushless);
    elevNeoMotor2 = new CANSparkMax(12, MotorType.kBrushless);

    elevNeoMotor1.restoreFactoryDefaults();
    elevNeoMotor2.restoreFactoryDefaults();

    elevNeoMotor1.setIdleMode(IdleMode.kBrake);
    elevNeoMotor2.setIdleMode(IdleMode.kBrake);

    RelativeEncoder encoder = elevNeoMotor1.getAlternateEncoder(8192);
    encoder.setPositionConversionFactor(5.498);
    elevNeoMotor1.getPIDController().setFeedbackDevice(encoder);

    elevNeoMotor1.setInverted(true);
    elevNeoMotor2.follow(elevNeoMotor1, true);
  }

  public void SetSpeed(double speed) {
    elevNeoMotor1.set(speed);
  }

  public Command moveElevatorUp() {
    return this.startEnd(() -> elevNeoMotor1.set(.1), () -> elevNeoMotor1.set(0));
  }

  public Command moveElevatorDown() {
    return this.startEnd(() -> elevNeoMotor1.set(-.1), () -> elevNeoMotor1.set(0));
  }
  public Command elevatorCoast() {
    return this.runOnce(() -> {
      this.elevNeoMotor1.setIdleMode(IdleMode.kCoast);
      this.elevNeoMotor2.setIdleMode(IdleMode.kCoast);
    });
  }
  
  public Command elevatorBrake() {
    return this.runOnce(() -> {
      this.elevNeoMotor1.setIdleMode(IdleMode.kBrake);
      this.elevNeoMotor2.setIdleMode(IdleMode.kBrake);
    });
  }
}
