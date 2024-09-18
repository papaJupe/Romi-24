// romi-traj-ramCmd - C					RamSettCmd.j 

// try extending RamseteCommand, use its super in constructor

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;

public class RamSettCmd extends RamseteCommand {
     // class var ?

  // constructor
public RamSettCmd(Trajectory path, Drivetrain train) {
  super(path,
  train::getPose,  // uses this for start +/- update?
  new RamseteController(2,0.7),
  new SimpleMotorFeedforward(
     0.2, // ksVolts, static coeff.
     10,  // kv VoltSecPerMeter ~maxV/max m./sec
     0),  // ka VoltSecondsSq/Meter ~maxV/max accel m/sec2
     train.mKinematics,
     train::getWheelSpeeds, // get actual wheel speed via encod.
    new PIDController(3.5, 0, 0.0),  //romi kP,I,D
    new PIDController(3.5, 0, 0.0), // L/R wheel
    // pass L/R wheel voltage to callback
    train::setVoltage,
    // require drive
    train);
// optional getController() to .setTolerance of veloc controlling PIDC
} // end constructor 

// periodic() update odometry done already in drive/auto periodic 
//  do I want/need init, execute, isFinished things for auto finish ? 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { // maybe test for @ final pose +/- or
    // timer expired?

    return false;
  }


} // end class
