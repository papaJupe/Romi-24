// RomiBase24 - A               drive subsystem    Drivetrain.j 

package frc.robot.subsystems;

// import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.756; // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark mLeftMotor = new Spark(0);
  private final Spark mRightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to DIO pins 4/5 and 6/7 for the left and right
  private final Encoder mLeftEncoder = new Encoder(4, 5);
  private final Encoder mRightEncoder = new Encoder(6, 7);

  // instance differential drive controller
  private final DifferentialDrive mDiffDrive =
      new DifferentialDrive(mLeftMotor, mRightMotor);

  // instance RomiGyro
  private final RomiGyro mGyro = new RomiGyro();

  // Setup BuiltInAccelerometer
  private final BuiltInAccelerometer mAccelerometer = new BuiltInAccelerometer();

  /** Creates a new Drivetrain. unclear purpose of Sendable Registry
   *  -- works fine without this
   */
  public Drivetrain() {
    // SendableRegistry.addChild(mDiffDrive, mLeftMotor);
    // SendableRegistry.addChild(mDiffDrive, mRightMotor);

    // We need to invert one side of the drivetrain so that (+) volts
    // result in both sides moving forward. Depending on how your
    // drive is arranged, you might have to invert the left side
  
    mRightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    mLeftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    mRightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();
  }  // end constructor

  // you don't want class method name identical to a super's
  public void arcaDriv(double yaxisSpeed, double zaxisRotate) {
    mDiffDrive.arcadeDrive(yaxisSpeed * 0.6, -zaxisRotate * 0.6, true);
  }

  public void resetEncoders() {
    mLeftEncoder.reset();
    mRightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return mLeftEncoder.get();
  }

  public int getRightEncoderCount() {
    return mRightEncoder.get();
  }

  public double getLeftDistanceInch() {
    return mLeftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return mRightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  /*
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return mAccelerometer.getX();
  }

  /**
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return mAccelerometer.getY();
  }

  /**
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return mAccelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   */
  public double getGyroAngleX() {
    return mGyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   */
  public double getGyroAngleY() {
    return mGyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   */
  public double getGyroAngleZ() {
    return mGyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    mGyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

} // end class
