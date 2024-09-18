// romi-traj-ramCmd-C
//class to make and store Traj's; when getMethod(name) recd, it should 
// return Traj obj with the named presets; another constructor when sent
// traj param (start, waypt, end), should return full Traj object

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

public final class TrajFarm {

    public static Trajectory traj;

    private final static double kTrackwidth = 0.141; // m.
    public final static DifferentialDriveKinematics mKinematics = new DifferentialDriveKinematics(kTrackwidth);

         // [instance used in TrajConfig v.i.]
    private final static DifferentialDriveVoltageConstraint autoVoltageConstraint =
                   new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(
                            0.2, // ksVolts, static coeff.
                            12, // kv VoltSecPerMeter ~maxV/max m./sec
                            0), // ka VoltSecondsSq/Meter ~maxV/max accel m/sec2
                    mKinematics, // DDK(trak width) v.s.
                    7); // [max V-1]

    // Create config for trajectory, instance used in TrajGene
    public static final TrajectoryConfig config = new TrajectoryConfig(
            0.7, // kMax SpeedMeters/Sec
            1) // kMax AccelMetersPerSecondSq
            // inverse kinematics transl chassis to wheel speed
            .setKinematics(mKinematics)// DDK(trakWidth)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint); // v.s. (SSFF, kDrivKine)

    // construct a new Trajectory obj called by name
    public TrajFarm(String name) {
        switch (name) {
            case "DRIV&TURN":
                // make the T manual entry of param x 4
                traj = TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two waypoints, making an 's' curve
                        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                        // End 3 meters straight ahead of start, facing forward
                        new Pose2d(3, 0, new Rotation2d(0)), config);
                break;
            case "SOMEFINGELSE":
                // make the T
                break;
            default:
                break;
        } // end switch block
    } // end construct

    public Trajectory getTraj() {
        return traj;

    }
} // end class