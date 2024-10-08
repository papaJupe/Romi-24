//  RomiRefeA-23                                    RomiGyro.j

package frc.robot.sensors;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Rotation2d;

public class RomiGyro {
  private SimDouble m_simRateX;
  private SimDouble m_simRateY;
  private SimDouble m_simRateZ;
  private SimDouble m_simAngleX;
  private SimDouble m_simAngleY;
  private SimDouble m_simAngleZ;

  private double m_angleXOffset;
  private double m_angleYOffset;
  private double m_angleZOffset;

  /** Create a new RomiGyro. */
  public RomiGyro() {
    SimDevice gyroSimDevice = SimDevice.create("Gyro:RomiGyro");
    if (gyroSimDevice != null) {
      gyroSimDevice.createBoolean("init", Direction.kOutput, true);
      m_simRateX = gyroSimDevice.createDouble("rate_x",
          Direction.kInput, 0.0);
      m_simRateY = gyroSimDevice.createDouble("rate_y",
          Direction.kInput, 0.0);
      m_simRateZ = gyroSimDevice.createDouble("rate_z",
          Direction.kInput, 0.0);

      m_simAngleX = gyroSimDevice.createDouble("angle_x",
          Direction.kInput, 0.0);
      m_simAngleY = gyroSimDevice.createDouble("angle_y",
          Direction.kInput, 0.0);
      m_simAngleZ = gyroSimDevice.createDouble("angle_z",
          Direction.kInput, 0.0);
    }
  } // end constructor

  /*
   * Get the rate of turn in degrees-per-second around the X-axis.
   * 
   * @return rate of turn in degrees-per-second
   */
  public double getRateX() {
    if (m_simRateX != null) {
      return m_simRateX.get();
    }
    return 0.0;
  }

  /*
   * Get the rate of turn in degrees-per-second around the Y-axis.
   * 
   * @return rate of turn in degrees-per-second
   */
  public double getRateY() {
    if (m_simRateY != null) {
      return m_simRateY.get();
    }
    return 0.0;
  }

  /*
   * Get the rate of turn in degrees-per-second around the Z-axis.
   * 
   * @return rate of turn in degrees-per-second
   */
  public double getRateZ() {
    if (m_simRateZ != null) {
      return m_simRateZ.get();
    }
    return 0.0;
  }

  /*
   * Get the currently reported angle around the X-axis.
   * 
   * @return current angle around X-axis in degrees
   */
  public double getAngleX() {
    if (m_simAngleX != null) {
      return m_simAngleX.get() - m_angleXOffset;
    }
    return 0.0;
  }

  /*
   * Get the currently reported angle around the X-axis.
   * 
   * @return current angle around Y-axis in degrees
   */
  public double getAngleY() {
    if (m_simAngleY != null) {
      return m_simAngleY.get() - m_angleYOffset;
    }
    return 0.0;
  }

  /*
   * Get the currently reported angle around the Z-axis.
   * 
   * @return current angle around Z-axis in degrees
   */
  public double getAngleZ() {
    if (m_simAngleZ != null) {
      return m_simAngleZ.get() - m_angleZOffset;
    }
    return 0.0;
  }

  /** Reset the gyro angles to 0. */
  public void reset() {
    if (m_simAngleX != null) {
      m_angleXOffset = m_simAngleX.get();
      m_angleYOffset = m_simAngleY.get();
      m_angleZOffset = m_simAngleZ.get();
    } // end if
  } // end reset

  public Rotation2d getRotation2d() {
    double deg = getAngleZ();
    return Rotation2d.fromDegrees(deg);
  }
} // end class
