// RomiBase 24  A                 auto cmd  DriveDistance.java


package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveDistance extends Command {
  private final Drivetrain mDrive;
  private final double mDistance;
  private final double mSpeed;

  /**
   * Creates a new DriveDistance - to drive your your robot a
   *  desired distance at desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem which this command runs
   */
  public DriveDistance(double speed, double inches, Drivetrain drive) {
    mDistance = inches;
    mSpeed = speed;
    mDrive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrive.arcaDriv(0, 0);
    mDrive.resetEncoders();
  }

  // Called every time the scheduler runs while it's scheduled.
  @Override
  public void execute() {  // no rotate
    mDrive.arcaDriv(mSpeed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrive.arcaDriv(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance moved from start to desired distance
    return Math.abs(mDrive.getAverageDistanceInch()) >= mDistance
;
  }
}  // end class
