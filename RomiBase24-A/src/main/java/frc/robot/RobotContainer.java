// // RomiBase24 -A       Robot Container. j

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoDistance;
import frc.robot.commands.TurnDegrees;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.romi.OnBoardIO;
import edu.wpi.first.wpilibj.romi.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. 
 * very little robot logic should be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).the specifics of 
 * this robot (subsystems, commands, and button mappings) config is 
 * made here .
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivetrain mDrivetrain = new Drivetrain();

  // Assumes a gamepad plugged into channel 0
  public static final XboxController mController = new XboxController(0);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> mChooser = new SendableChooser<>();

  private final OnBoardIO mOnboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);
  // NOTE: The I/O pin functional of the 5 exposed I/O pins depends on the hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
  // By default, the following are available (listed in order from inside of the board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

  /** The specifics for this robot. Defines subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /*Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    // Default command is ArcadeDrive. This will run unless another
    // command is scheduled over it.
    mDrivetrain.setDefaultCommand(new ArcadeDrive(mDrivetrain));

    // Example of how to use the onboard IO
    Trigger onboardButtonA = new Trigger(mOnboardIO::getButtonAPressed);
    onboardButtonA
        .onTrue(new PrintCommand("Button A Pressed"))
        .onFalse(new PrintCommand("Button A Released"));

    // config SmartDashboard options to set auto
    mChooser.setDefaultOption("Auto Drive Dist", new AutoDistance(mDrivetrain));
    mChooser.addOption("AutoTurn180", new TurnDegrees(0.9, 180, mDrivetrain));
    SmartDashboard.putData(mChooser);
  }

  /* passes the autonomous command to the main {@link Robot} class
   */
  public Command getAutonomousCommand() {
    return mChooser.getSelected();
  }

  // Used before simplifications to specify default cmd for teleop
   
  // public Command getArcadeDriveCommand() {
  //   return new ArcadeDrive(
  //       m_drivetrain, () -> -m_controller.getRawAxis(1), () -> -m_controller.getRawAxis(2));
  // }
} // end RC class
