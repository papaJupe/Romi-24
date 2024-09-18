//  romi-traj             RobotContainer.java
// this project shifts all config, butt bind... to Robot.j
package frc.robot;

import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoDistSequ;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic 
 * should be handled in the {@link Robot} periodic methods 
 * (other than the scheduler calls). 
 * Instead, the specifics of operating the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RCstub {
    // instance the two subsystems
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT,
        ChannelMode.INPUT);
        
  // NOTE: I/O pin functionality of the 5 exposed I/O pins depends on 
  // the hardware "overlay" that is specified in settings of the wpilib-ws
  // server (web interface) on the Romi's raspberry Pi.
  // By default, the following are available, from board inside to out:
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  // Your subsystem configuration should take the overlays into account
  
  // instance joystick --assumes stick or gamepad plugged into USB 0
  public final static Joystick m_controller = new Joystick(0);

  // allows SmartDashboard to pick autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * CONSTRUCT container for robot: its single method, configBB(), sets
   * Drivetrain [subsystem's] default Cmd, OperatorInterface (OI) actions,
   * Smart Dashbd Autonomous chooser options.
   *  --- defines the specifics of this robot
   */
  public RCstub() {
    // Configure joystick button bindings et. al.
    configureButtonBindings();
  } // end constructor

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
    ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), then 
    * passing it to a JoystickButton.
   */
  private void configureButtonBindings() {
    // Default command now ArcadeDrive. Runs unless another command
    // is scheduled over it.(e.g. runs in teleOp unless overridden)
    //[orig.] m_drivetrain.setDefaultCommand(getArcadeDriveCommand());
    //[now] clearer syntax uses simpler constructor for ArcadeDrive()
     m_drivetrain.setDefaultCommand(new ArcadeDrive(m_drivetrain));
    
     // Example of onboard IO buttons doing something
    Trigger onboardButtonA = new Trigger(m_onboardIO::getButtonAPressed);
      onboardButtonA
         .onTrue(new PrintCommand("Button A Press"))
         .onFalse(new PrintCommand("Button A Release"));

    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Auto Distance", new
                  AutoDistSequ(m_drivetrain));
    // m_chooser.addOption("Auto Timed", new AutonomousTime(m_drivetrain));
    SmartDashboard.putData(m_chooser);

  }  // end configBB()

  // ... passes selected auto command to the scheduling {@link Robot} class.
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }  // end get.AutoCmd

  /* [orig. was] 'Use this to pass the teleop command to the main Robot
   * class.' <--confusing because AD Cmd never actually appears in Robot.j
   * explicitly. And not needed for simple ArcadeDrive constructor now used.
   */
  // public Command getArcadeDriveCommand() {
  //   return new ArcadeDrive(
  //       m_drivetrain, () -> -m_controller.getRawAxis(1), () ->
   //  m_controller.getRawAxis(2)); // better control to get turn speed
  // }                             // from Rt. stick X axis (4)

} // end RC class
