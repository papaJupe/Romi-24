
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;

public class RamSetCMD2 extends Command {
  private Trajectory traj;
  private Drivetrain train;

  // constructor
  public RamSetCMD2(Trajectory path, Drivetrain chassis) {
    traj = path;
    train = chassis; // vals passed to generator v.i., ? could go here
    generateRamsCmd();
  } // end constructor

  public Command generateRamsCmd() {
    RamseteCommand ramSet = new RamseteCommand( // takes 10 param
        traj, train::getPose, // uses this for start +/- update?
        new RamseteController(2, 0.7),
        new SimpleMotorFeedforward(
            0.2, // ksVolts, static coeff.
            10, // kv VoltSecPerMeter ~maxV/max m./sec
            0), // ka VoltSecondsSq/Meter ~maxV/max accel m/sec2
        train.mKinematics,
        train::getWheelSpeeds, // translate chassis params to wheel speed
        new PIDController(3.0, 0, 0.00), //  kP,I,D
        new PIDController(3.0, 0, 0.00),
        // pass L/R wheel volt to callback
        train::setVoltage,
        train
        );
    // optional getController() to .setTolerance on PIDC

    return ramSet;
  } // end generator

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

} // end class
