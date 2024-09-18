// few PID numbers used in romiTrajCmd - C -24

package frc.robot;

/*
 * It is advised to statically import this class (or one of its inner
 * classes) wherever constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        //   public static final int kLeftMotor1Port = 0;
        //   public static final int kLeftMotor2Port = 1;
        //   public static final int kRightMotor1Port = 2;
        //   public static final int kRightMotor2Port = 3;
      
        //   public static final int[] kLeftEncoderPorts = new int[] {0, 1};
        //   public static final int[] kRightEncoderPorts = new int[] {2, 3};
        //   public static final boolean kLeftEncoderReversed = false;
        //   public static final boolean kRightEncoderReversed = true;
      
        //   public static final int kEncoderCPR = 1024;
        //   public static final double kWheelDiameterInches = 6;
        //   public static final double kEncoderDistancePerPulse =
        //    // Assumes the encoders are directly mounted on the wheel shafts
        //       (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;
      
        //   public static final boolean kGyroReversed = false;
      
          public static final double kStabilizP = 0.03;
          public static final double kStabilizI = 0.0;
          public static final double kStabilizD = 0.0;
      
          public static final double kTurnP = 0.0015;
          public static final double kTurnI = 0.001;
          public static final double kTurnD = 0.0001;
      
          public static final double kMaxTurnVeloc = 60;
          public static final double kMaxTurnAcceler = 60;
      
          public static final double kTurnTolerDeg = 1;
          public static final double kTurnRateTolerVeloc = 5; 
          // deg per second
        }  // end drive constant
}
