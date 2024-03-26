package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;

public class DriveToNote extends Command {
  private final Chassis chassis;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");

  private final PIDController translationController = new PIDController(0.07, 0, 0);
  private final PIDController rotationController = new PIDController(0.07, 0, 0);

  /** Creates a new FieldOrientedDrive. */
  public DriveToNote(Chassis chassis) {
    this.chassis = chassis;

    translationController.setSetpoint(0.0);
    rotationController.setSetpoint(0.0);

    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    translationController.reset();
    rotationController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double dx = tx.getDouble(0.0);
    double dy = ty.getDouble(0.0);

    double translation = translationController.calculate(dy);
    double rotation = rotationController.calculate(dx);

    chassis.driveRobotRelative(new ChassisSpeeds(-translation, 0, rotation));
  }
}
