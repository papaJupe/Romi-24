// romi-traj-ramCmd - C	24	                        	Robot.j 
// test in progress
// 1st try traj follow using mod. of cmd/subsys fmwrk. Adapted from
// rtr-B -- control/config in Robot.j, minimal lambda, Constant moved
// to where used. tried PID controlled wheel rates to .drive (fwd,rot)
// in both stick control, and plain auto (drive dist) both very curvy
// so replaced with dD.arcaDrive and button activated PID for straight
// teleOp drive. Both traj Cmds fail to do anything, need scheduling ?
// auto stabilized by subclassing PIDCommand ->DriveDistaStabl

// For live vision, attach camera to any pi port; its cam server streams
// automatically to pi web interface: wpilibpi.local:1181 or mpeg strm,
// and (when Sim is running) to Shuffleboard.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
//import edu.wpi.first.math.util.Units;
import java.util.List;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.DriveDistaStabl;
import frc.robot.commands.RamSetCMD2;
import frc.robot.commands.RamSettCmd;
import frc.robot.sensors.RomiGyro;

//   -- everything configured and run from here  
public class Robot extends TimedRobot {

  // instance joystick @ 0 -- i.e. controller plugged into USB 0
  public final static XboxController mController =
                                   new XboxController(0);
  // public final Joystick mController = new Joystick(0);

  private final OnBoardIO mOnboardIO = 
                    new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);

  public final Drivetrain mDrive = new Drivetrain();

  public static RomiGyro mGyro = new RomiGyro();

  private final SendableChooser<String> chooser = new SendableChooser<>();

  private String autoSelected; // string used by autoPeriodic

  private Command autoComm = null;

  // An example trajectory for autonomous period
  private Trajectory trajMulti; // track around 2 pins
  // another
  private Trajectory trajLoop; // single CW loop

  // roboInit runs when the robot is first started and sets
  // this robot's specifics (things done in RC in large projects)
  @Override
  public void robotInit() {
    mDrive.setDefaultCommand(new ArcadeDrive(mDrive));

    // Example of onboard IO buttons doing something
    Trigger onboardButtonA = new Trigger(mOnboardIO::getButtonAPressed);
    onboardButtonA /// requires RP to call scheduler
        .whileTrue(new PrintCommand("onbord A Press"))
        .whileFalse(new PrintCommand("onbord A Release"));

    // To drive straight w/ gyro when R bumper is held in teleOp
    new JoystickButton(mController, 6)
    .whileTrue(
        new PIDCommand(
            new PIDController(0.035, 0.0, 0.0),
            // Close the loop using turn rate/angle
            mGyro::getAngleZ,
            // Setpoint is 0 for straight driving
            0,
            // Pipe output to the turning param of diffDriv method
            output -> mDrive.arcaDriv(-mController.getLeftY()
                  * 0.8, -output),
            // Require robot drive (must be a subsyst)
            mDrive));    

    chooser.setDefaultOption("plainDrivDist", "DRIVDIST");
    chooser.addOption("single loop", "DRIV&TURN");
    chooser.addOption("navigate2pin", "MULTIPT");

    SmartDashboard.putData("AutoSelect ", chooser);

   // Create the trajectory for autonomous. It's best to initialize
    // trajectories here to avoid wasting time in autonomous.
    // single CW turn
    trajLoop = TrajectoryGenerator.generateTrajectory(// all distance in m.
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)), //start (N)
        List.of(new Translation2d(0.25, 0.15),
        new Translation2d(0.75, 0.35),
        new Translation2d(1.5, 0.0),
        new Translation2d(0.75, -0.35)), // waypt's m., ccw = (+)
        new Pose2d(0.0, -0.0, Rotation2d.fromDegrees(90)), // end
        // CW circle finishes pointing west
        new TrajectoryConfig(Units.feetToMeters(1.0), // veloc
                             Units.feetToMeters(0.25))); // accel

    trajMulti = TrajectoryGenerator.generateTrajectory( // distance in m.
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)), // start (N)
        List.of(new Translation2d(0.25, 0.1),
            new Translation2d(0.75, 0.35),
            new Translation2d(1.25, 0),
            new Translation2d(1.75, -0.35),
            new Translation2d(2.25, 0.0),
            new Translation2d(1.75, 0.35),
            new Translation2d(1.25, 0.0),
            new Translation2d(0.75, -0.35)),
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90)), // end
        // should finish pointing +90, West
        new TrajectoryConfig(Units.feetToMeters(1.0),
            Units.feetToMeters(0.25))); // end genTraj
    // TC's param 0 is maxSpeed--m/sec, param 1 is maxAccel--m/secSq
    // cmd code add .setKinematic & .addConstraint method to new TC obj

  } // end robotInit()

  // called every robot packet, no matter the mode.
  @Override
  public void robotPeriodic() { // call CmdScheduler for any to work
    // ...in flat framework will run essential stuff regardless of mode
    if (mController.getRawButton(1)) {// [xbox button A]
      mGyro.reset();
      mDrive.mLeftEncoder.reset();
      mDrive.mRightEncoder.reset();
    }
    if (mController.getRawButton(2)) // [xbox button B]
      System.out.println("present Pose = " + mDrive.getPose());

    SmartDashboard.putNumber("Z axis Rot", mGyro.getAngleZ());

    CommandScheduler.getInstance().run();

  } // end robotPeriodic

  // autoInit gets the autonomous 'command String' set by SmartDashbd
  @Override
  public void autonomousInit() {
    // RC got selected auto from SmartDashboard as CMD, here just String
    autoSelected = chooser.getSelected();
    mDrive.mLeftEncoder.reset();
    mDrive.mRightEncoder.reset();
    mGyro.reset();

    // will use switch/case so selected string sends premade traj's
    // to RamSettCmd to define autoSelectCmd, then schedule that here
    switch (autoSelected) {
      case "DRIVDIST":
        autoComm = new DriveDistaStabl(0.7, 1, mDrive);
        break;

      case "DRIV&TURN":// later-- auto strings send various traj to
        // trajfarm, then call RamSettCmd with those
        autoComm = new RamSettCmd(trajLoop, mDrive);
        break;

      case "MULTIPT":
        autoComm = new RamSetCMD2(trajMulti, mDrive);
        break;
      default:
        break;

    } // end switch block

    // Reset the drivetrain odometry to the starting pose of all traj.
    // -- should be there already
       mDrive.resetOdometry(trajLoop.getInitialPose()); // (0,0,0)
    if (autoComm != null)
      autoComm.schedule();
  } // end autoInit

  @Override
  public void autonomousPeriodic() {
    // Update odometry was called in subsyst periodic() in cmd example
    mDrive.updateOdometry(); // only call to it in this program
    // if (autoComm != null)
    // autoComm.schedule();

  } // end autoPeriod

  @Override
  public void teleopInit() {
    // This ensures that the autonomous code has stopped
    if (autoComm != null)
      autoComm.cancel();
    mDrive.mLeftEncoder.reset();
    mDrive.mRightEncoder.reset();
    mGyro.reset();

  } // end teleInit

  @Override // can leave empty to enable default cmd, ArcDrv in teleOp
  public void teleopPeriodic() { 
    // no role for traj. calc, but PID controlled
    // .drive method worked very badly
    // Get the fwd (x) speed, inverting because most controllers
    // return negative value when stick's forward (Y axis for stick var).
//     final var xSpeed = -mDrive.mSpeedLimiter.calculate // apply slew
//        (mController.getRawAxis(1)) * Drivetrain.kMaxSpeed;
//  //    (m_controller.getLeftY()) * Drivetrain.kMaxSpeed;
//     // Get the rate of angular rotation: inverting a stick's neg value to
//     // a positive value to turn left, CCW -- remember, CCW is positive in
//     // rotation2d. Xbox controller returns positive values when you pull
//     // stick right by default.
//     final var rot = //-mDrive.mRotLimiter.calculate // (no slew)
//           -(mController.getRightX()); // gamepad * Drivetrain.kMaxAngularSpeed
//       // (m_controller.getRawAxis(0)) * Drivetrain.kMaxAngularSpeed;
  
//     mDrive.drive(xSpeed, rot);
  } // end teleoPeriod; send 2 param to PID assisted .drive() method

  // called once when robot enters Disabled mode.
  @Override
  public void disabledInit() {
    // System.out.println("presentPose= " + mDrive.getPose());
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
  }

  /** called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
} // end robot class