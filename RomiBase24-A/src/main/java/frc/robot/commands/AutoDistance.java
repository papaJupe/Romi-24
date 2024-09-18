// RomiBase24 - A              commands/AutoDistance.java

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoDistance extends SequentialCommandGroup {
  /**
   * construct new Autonomous sequence using encoder-based distance.
   * drive a specified distance, turn 180 deg, (could) drive back &
   *  turn again if turns accurate.
   * @param drivetrain the subsystem to be controlled
   */
  public AutoDistance(Drivetrain drivetrain) {
    addCommands(
        new DriveDistance(1.0, 36, drivetrain),
        new WaitCommand(4.0),
         new TurnDegrees(0.8, -180, drivetrain),
        new WaitCommand(4.0),
        new DriveDistance(1, 36, drivetrain),
        new WaitCommand(4.0),
        new TurnDegrees(0.8, 180, drivetrain));
  }
}  // end class
