// romiGyroDrivPID - C  24 		       DriveDistaStable auto cmd

// subclassing PIDCommand seems to give more stable predictable
// control of drive stabliz and turns than freestanding PIDC as in v.D
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import static frc.robot.Robot.mGyro;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveDistaStabl extends PIDCommand {
  // private final double m_speed;
  private final double mDistance;
  private final Drivetrain mDrive;

  /** Construct a new DriveDistaStable cmd */
  public DriveDistaStabl(double speed, double meter, Drivetrain drive) {
    super(
        new PIDController(
            DriveConstants.kStabilizP,
            DriveConstants.kStabilizI,
            DriveConstants.kStabilizD),
        // feedback using the turn rate/ or actual angle?
        mGyro::getAngleZ,
        // Setpoint is 0, v.i. initialize()
        0.0,
        // Pipe the output to the turning cmd
        output -> drive.arcaDriv(speed, -output),
        // Require the robot drive
        drive); // end super()
    // m_speed = speed;
    mDistance = meter;
    mDrive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here
    // the delta tolerance ensures the robot is stationary at the
    // setpoint before it's considered to reach the reference
    getController().setTolerance(DriveConstants.kTurnTolerDeg,
                                DriveConstants.kTurnRateTolerVeloc);
  } // end constructor

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrive.arcaDriv(0, 0);
    mDrive.resetEncoders();
    mGyro.reset();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrive.arcaDriv(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    return Math.abs(mDrive.getAverageDistanceM()) >= mDistance;
  }
} // end class
