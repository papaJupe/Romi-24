// romi-traj-ramCmd-C              commands/AutoDistQuence.java
// not used in v. C - 24
package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoDistSequ extends SequentialCommandGroup {
  /* construct new Autonomous sequence using encoder readings.
   * drive a specified distance, turn 180 deg, drive back and turn
   * again if turns accurate enough for this.
   * @param drivetrain the subsystem to be controlled
   */
  public AutoDistSequ(Drivetrain drivetrain) {
    addCommands(
        new DriveDistance(0.5, 1.5, drivetrain),
        new WaitCommand(4.0),
        new TurnDegrees(0.5, 180, drivetrain),
        new WaitCommand(4.0),
        new DriveDistance(0.5, 1.5, drivetrain),
        new WaitCommand(4.0),
        new TurnDegrees(0.5, -180, drivetrain));
  }
}  // end class
