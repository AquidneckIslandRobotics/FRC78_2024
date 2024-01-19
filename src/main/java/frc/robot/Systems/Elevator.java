// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private CANSparkMax ElevatorNeo;
  /** Creates a new Elevator. */
  public Elevator() {
    ElevatorNeo = new CANSparkMax(17, MotorType.kBrushless);
  }

  public void setElevatorSpeed(double speed){
    ElevatorNeo.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
