// romi-traj-ramCmd-C                          ArcadeDrive.j cmd
// edited so overloaded constructor allows orig. lambda
//  (with minor edit) or simpler syntax

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import  frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.Command;
//import java.util.function.Supplier;

public class ArcadeDrive extends Command {
  private final Drivetrain mDrive; 
  // a local pvt variable for this class

  // private final Supplier<Double> m_xaxisSpeedSupplier;
  // private final Supplier<Double> m_zaxisRotateSupplier;

  private double fwdSpeed;
  private double turnSpeed;
  private boolean isLamda;

  /**
   * orig. complex constructor was:
   * Creates a new ArcadeDrive. This command will drive according to
   * the speed supplier lambdas. This command does not terminate.
   * [@param] drivetrain The drivetrain subsystem this command will run
   * [@param] xaxisSpeedSupplier Lambda supplier of forward/backward speed
   * [@param] zaxisRotateSupplier Lambda supplier of rotational speed
   */

  // revised constructor gets one input param
  public ArcadeDrive(Drivetrain drivetrain) {
    mDrive = drivetrain;
    isLamda = false; // when using simple syntax below
    addRequirements(drivetrain);
  }  

  // overloaded constructor needs to have common var for diffDrive's
  // arcadeDriv() method;
  // public ArcadeDrive(
  //      Drivetrain drivetrain,
  //      Supplier<Double> xaxisSpeedSupplier,
  //      Supplier<Double> zaxisRotateSupplier) {

  //  m_drivetrain = drivetrain;
  //  Supplier<Double> m_xaxisSpeedSupplier = xaxisSpeedSupplier;
  //  Supplier<Double> m_zaxisRotateSupplier = zaxisRotateSupplier;
  //  isLamda = true;
  // addRequirements(drivetrain);
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // mDrive.resetEncoders();// done in teleOpInit, need here too? N
  }

  // Called every time the scheduler runs while command is scheduled.
  @Override
  public void execute() {
    if (isLamda) {}// set speed var w/ suppliers
    // m_drivetrain.arcadeDrive(m_xaxisSpeedSupplier.get(),
    // m_zaxisRotateSupplier.get()); else, get stick values
    else {
    // easy to understand control using joystick Y and X axes
    // invert left stick Y
    fwdSpeed = -Robot.mController.getRawAxis(1) * Drivetrain.kMaxSpeed; 
    // also invert rot direction because CW is neg for Math._ classes 
    turnSpeed = -Robot.mController.getRawAxis(4) * Drivetrain.kMaxAngularSpeed; 
    // gamepad's X is 4
    }
    // mDrive.drive(fwdSpeed, turnSpeed ); // .drive() has PID to regulate wheel
    // speed to these values x2, but no feedback to control angle of travel
    // and awful control resulted; usual AD worked much better
    mDrive.arcaDriv(fwdSpeed, turnSpeed );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // default cmd does not finish when interrupted
  }
} // end class

