// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  XboxController controller = new XboxController(0);
  
   Translation2d frontLeft = new Translation2d(1, 1);
   Translation2d frontRight = new Translation2d(-1, 1);
   Translation2d backLeft = new Translation2d(1, -1);
   Translation2d backRight = new Translation2d(-1, -1);

   SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeft, frontRight, backLeft, backRight);

   SwerveModule frontLeftModule = new SwerveModule(0, 4); // (0, 4)
   SwerveModule frontRightModule = new SwerveModule(1, 5);
   SwerveModule backLeftModule = new SwerveModule(2, 6);
   SwerveModule backRightModule = new SwerveModule(3, 7);

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double xSpeed = controller.getLeftX(); // -1 .. +1 
    double ySpeed = controller.getLeftY(); // -1 .. +1
    double rotationSpeed = controller.getRightY(); // -1 .. +1

    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

    frontLeftModule.drive(states[0]);
    frontRightModule.drive(states[1]);
    backLeftModule.drive(states[2]);
    backRightModule.drive(states[3]);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
