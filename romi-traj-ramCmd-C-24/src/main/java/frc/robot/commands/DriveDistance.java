// romi-traj-ramCmd-C  24          auto cmd  DriveDistance.java
// not used here -- very curvy, replaced by DriveDistaStabl

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveDistance extends Command {
  private final Drivetrain m_drive;
  private final double m_distance;
  private final double m_speed;

  /**
   * CONSTRUCT a new DriveDistance. This command drives robot for
   * a set distance at a set speed, theoretically straight.
   * @param speed The speed at which the robot is to drive
   * @param inches The number of inches the robot should drive
   * @param drive The drivetrain subsystem for this command 
   */
  public DriveDistance(double speed, double meters, Drivetrain drive) {
    m_speed = speed;
    m_distance = meters;
    m_drive = drive;
    
    addRequirements(drive);
  } // end constructor

//   Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcaDriv(0.0, 0.0);
    m_drive.resetEncoders();
   
  }

//   Called every loop while the command is scheduled.
  @Override
  public void execute() {//should use PIDcontroller but seems not;
  // drives very skewed left
    m_drive.arcaDriv(m_speed, 0);
  }

//   // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcaDriv(0, 0);
  }

//   // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    return Math.abs(m_drive.getAverageDistanceM()) >= Math.abs(m_distance);
  }
} // end DD command
