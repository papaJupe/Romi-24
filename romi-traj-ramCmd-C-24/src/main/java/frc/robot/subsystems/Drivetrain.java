//  Romi-traj-rams-C         drive subsystem    Drivetrain.j   

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

import static frc.robot.Robot.mGyro;
import frc.robot.Robot;

public class Drivetrain extends SubsystemBase {
  private static final double kTrackWidth = 0.141; // meters
  private static final double kWheelDiaMeter = 0.070; // = 2.755"
  private static final int kCountsPerRevo = 1440;

  // kMax__ constant only used to multiply stick input in Robot.j
  public static final double kMaxSpeed = 0.7; // meters per second
  public static final double kMaxAngularSpeed = 0.5; // rad/sec

  // Slew rate limiter --> ramp stick inputs, take 1/3 sec from 0 to 1
  public final SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(3);
  public final SlewRateLimiter mRotLimiter = new SlewRateLimiter(5); // v fast

  // Romi has the left and right motors on PWM channels 0 and 1
  private final Spark mLeftMotor = new Spark(0);
  private final Spark mRightMotor = new Spark(1);

  // The Romi's onboard encoders are hardcoded to DIO pins 4/5 and 6/7
  public final Encoder mLeftEncoder = new Encoder(4, 5);
  public final Encoder mRightEncoder = new Encoder(6, 7);

  // private final PIDController mLeftPIDController = new PIDController(5.0, 0.01,
  // 0.0);
  // private final PIDController mRightPIDController = new PIDController(5.0,
  // 0.01, 0.0);

  // public static RomiGyro m_gyro = new RomiGyro();
  // was here, now instance the RomiGyro in Robot.j

  private final DifferentialDrive mDiffDrive = new DifferentialDrive(mLeftMotor, mRightMotor);

  // DDK class enables conversion of incoming speed/rot param (chassis) to
  // diffDrive R/L wheel speeds, like what DD's arcadeDrive() does;
  // used in .drive method v.i.
  public final DifferentialDriveKinematics mKinematics = new DifferentialDriveKinematics(kTrackWidth);

  // DDO class holds current field Pose2d; initialized in class Constructor;
  private final DifferentialDriveOdometry mOdometry;

  // Gains must be set for each robot! 2 param constructor uses kS, kV
  // private final SimpleMotorFeedforward m_feedforward = new
  // SimpleMotorFeedforward(0.2, 10.0);
  // cmd version's RC has 3 param kS kV kA & addConstraint for TrajConfig
  // instance. here m_feedfwd used only in setSpeed() method below

  // has differential-drive arcadeDrive() method for stick control
  // + other drive mechanism based classes needed for ramsCmd to use it
  // Sets encoder distance/ pulse, resets gyro, creates a DDO instance
  public Drivetrain() {
    // invert one side of the drivetrain so that positive voltages
    // keep both motors moving forward.
    mRightMotor.setInverted(true);

    // Set the distance per pulse for the encoders -- simply use the
    // distance of one wheel rot. [meter] divided by encoder resolution
    mLeftEncoder.setDistancePerPulse(Math.PI * kWheelDiaMeter / kCountsPerRevo);
    mRightEncoder.setDistancePerPulse(Math.PI * kWheelDiaMeter / kCountsPerRevo);

    mLeftEncoder.reset();
    mRightEncoder.reset();
    mGyro.reset();

    // DDO instance made here holds initial Pose2d: field centric X,Y
    // (in m.), angle Rot2d; input param current Rot2d (rad.), encod
    // .getDistance() x2 returns values in m. . I assume this
    // DDO 3 param constructor adds Pose (0,0,0) by default ?
    // 4 param constructor probably allows different initial field Pose
    mOdometry = new DifferentialDriveOdometry(
        Robot.mGyro.getRotation2d(),
        mLeftEncoder.getDistance(),
        mRightEncoder.getDistance());
    // drive's DDO methods work like get() set() when called from
    // Robot.j

  } // end constructor

  // used only for stick control and non-traj autos, all suppliers 
  // inverted rot sign already
  public void arcaDriv(double xSpeed, double rot) {
    mDiffDrive.arcadeDrive(xSpeed, rot);
  }

        // small feedback PID numbers work better if not squared
        public void arcaDrivP(double xaxisSpeed, double zaxisRotate) {
        
          mDiffDrive.arcadeDrive(xaxisSpeed , zaxisRotate, false);
        }
  // (stick) used PID to keep R/L speeds regulated in setSpeed(),
  // feeding voltage directly to motors, for fwd speed and rotation
  // -- worked badly -- needs gyro correction
  // v. romi gyro PID C to add

  // .drive method used by robot.j's teleoPerio (stick) and plain Auto cmd,
  // not by rams Cmds, which seems to package the multi-param steps it used
  // and send params right to ...

  public void setVoltage(double left, double right) {
    mLeftMotor.setVoltage(left);
    mRightMotor.setVoltage(right);
  }

  public void resetEncoders() {
    mLeftEncoder.reset();
    mRightEncoder.reset();
    // mGyro.reset();
  }

  public double getAverageDistanceM() {
    return (mLeftEncoder.getDistance() + mRightEncoder.getDistance()) / 2;
  }

  // 1st step in drive() v.s.; cf method below using encoders to make the DDWS
  // obj.
  // public DifferentialDriveWheelSpeeds getWheelSpeeds(double xSpeed, double rot)
  // {
  // wheelSpeeds = mKinematics.toWheelSpeeds
  // (new ChassisSpeeds(xSpeed, 0.0, rot));
  // return wheelSpeeds; } // inverse kinemat calc here, translate field relative
  // desired move (from operator stick) to robot's mechanical wheel speeds

  // @return The current wheel speeds for ramsete calcul of next pos.
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(mLeftEncoder.getRate(),
        mRightEncoder.getRate());
  }

  /*
   * Updates the field-relative position to current;
   * it's getting robot-centric data from encoders,
   * so must be estimating field pos from integrating
   * these and angles traveled ? Called by Robot.j's
   * autoPeriodic() only; constantly updates ramseteController calculation
   */
  public void updateOdometry() {
    mOdometry.update(
        mGyro.getRotation2d(),
        mLeftEncoder.getDistance(),
        mRightEncoder.getDistance());
  }

  // Resets the field-relative position to a specific new base position
  // on field (fieldX, fieldY, field Rota2d) (method internally zeros the
  // Rota2d setting)
  // in rams-Cmd example of this he resets encoders also to 0. Here
  // only used in robot.j autoInit to initialize @ 0,0,0 or base field pos
  // but if new Pose2d param provided by external source, could make course
  // correction to improve ramsete Cmd accuracy.
  public void resetOdometry(Pose2d pose) {
    mOdometry.resetPosition(
        mGyro.getRotation2d(),
        mLeftEncoder.getDistance(),
        mRightEncoder.getDistance(),
        pose);
  }

  // print var's /w getPose to see what m_odometry holds after travel
  // -- if it's actually calculating field pos from encoder input;
  // could display on SmtDashbd too -- done here in roboPeriodic
  // @return The pose of the robot currently held in odometry instance
  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }
} // end class
