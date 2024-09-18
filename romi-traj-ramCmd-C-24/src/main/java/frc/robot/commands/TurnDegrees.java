// romi-traj-rams-C               auto cmd       TurnDegrees cmd
// crude base methods, could be edited to use gyro, PID ...
// not used in v. 24
package frc.robot.commands;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class TurnDegrees extends Command {

  private final double m_speed;
  private final double m_degrees;
  private final Drivetrain mDrive;

  /* CONSTRUCT new TurnDegrees. Command will turn Romi for X degr.
   * calculated from encoders report of wheel travel.
   * @param speed -- of z-axis turn, + CW, negative is CCW here.
   * param degrees - to turn, using R/L wheel encoders to measure rotation.
   * @param drive The drive subsystem to use */
   public TurnDegrees(double speed, double degrees, Drivetrain drive) {
    m_speed = speed; // ==> rotation speed, pos val should turn CW
    m_degrees = degrees;
    mDrive = drive; 
     addRequirements(drive);
   }  // end constructr

   // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, set encoders to zero
    mDrive.arcaDriv(0, 0);
    mDrive.resetEncoders();
  }
  
  // Called every time cmd scheduler runs while the command is scheduled.
 @Override
  public void execute() {
    mDrive.arcaDriv(0, m_speed);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrive.arcaDriv(0, 0);
  }
  
  @Override     // Returns true when the command should end.
  public boolean isFinished() {
    /* Need 2 convert bot degree rotation to distance req. for wheels 
    each to rotate. Romi chazssis
    https://www.pololu.com/category/203/romi-chassis-kits
    has a wheel placement diameter (149 mm) - width of wheel (8 mm)
     = 141 mm or 5.551 inches = ctr to ctr. One full robot rotation,
      360 deg, needs pi * d = 17.44 in., each opposed wheel to rotate.
    */
    double inchPerDegree = Math.PI * 5.4 / 360; // less to correct over-turn
    //Compare distance turned from start to distance needed for N 
    // degrees.
    return getAverageTurningDistance() >= (inchPerDegree * m_degrees);
  }

  private double getAverageTurningDistance() {
    double leftDistance = Math.abs(mDrive.mLeftEncoder.getDistance());
    double rightDistance = Math.abs(mDrive.mRightEncoder.getDistance());
    return (leftDistance + rightDistance) / 2.0;
  }
}  // end class

