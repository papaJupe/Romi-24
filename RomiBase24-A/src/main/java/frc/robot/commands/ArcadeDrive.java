// romiBase24 - A                     ArcadeDrive.j cmd
// edited so overloaded constructor allows orig. lambda (with minor edit)
// or simpler syntax now in RobotContainer

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;
//import java.util.function.Supplier;

public class ArcadeDrive extends Command {
  private final Drivetrain drive; // this is just a local pvt
  // variable for this class

  // private final Supplier<Double> m_xaxisSpeedSupplier;
  // private final Supplier<Double> m_zaxisRotateSupplier;

  private double fwdSpeed;
  private double turnSpeed;
  private boolean isLamda;

  /**
   * orig. complex constructor had:
   * Creates a new ArcadeDrive. This command will drive according to
   * the speed supplier lambdas. This command does not terminate.
   * [@param] drivetrain The drivetrain subsystem this command will run
   * [@param] xaxisSpeedSupplier Lambda supplier of forward/backward speed
   * [@param] zaxisRotateSupplier Lambda supplier of rotational speed
   */

  // revised constructor gets one simple input param
  public ArcadeDrive(Drivetrain drivetrain) {
    drive = drivetrain;
    isLamda = false; // to set options v.i.
    addRequirements(drivetrain);
  }  

  // overloaded constructors need to have common var to send to diffDrive's
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
    drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isLamda) {}     // set speed var w/ suppliers
    // m_drivetrain.arcadeDrive(m_xaxisSpeedSupplier.get(),
    // m_zaxisRotateSupplier.get()); else, get stick values
    else {
    // easy to understand control using joystick's Y and X axes
    fwdSpeed = -mController.getRawAxis(1); // left stick Y 
    turnSpeed = mController.getRawAxis(4); // rt gamepad X is 4
    }
    drive.arcaDriv(fwdSpeed, turnSpeed );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // default does not finish when interrupted
  }
} // end class

