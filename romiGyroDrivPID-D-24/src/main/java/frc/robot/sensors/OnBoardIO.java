// romiGyroDrivPID D-24     
// allows coder to set mode and operation of DIO 1 & 2
// could be subsyst subclass, not sure why / not ?

package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class represents the onboard IO of the Romi ; 
 * configures some of the pushbuttons and LEDs.
 *
 * DIO 0 - Button A (input only) DIO 1 - Button B (input) or Green LED
 *  (output)
 * DIO 2 - Button C (input) or Red LED (output) DIO 3 - Yellow LED 
 * (output only)
 */
public class OnBoardIO  { //extends SubsystemBase
  private final DigitalInput m_buttonA = new DigitalInput(0);
  private final DigitalOutput m_yellowLed = new DigitalOutput(3);

  // DIO 1
  private DigitalInput m_buttonB;
  private DigitalOutput m_greenLed;

  // DIO 2
  private DigitalInput m_buttonC;
  private DigitalOutput m_redLed;

  private static final double MESSAGE_INTERVAL = 1.0;
  private double m_nextMessageTime;

  public enum ChannelMode {
    INPUT,
    OUTPUT
  }

  /**
   * Constructor
   *
   * @param dio1 Mode for DIO 1 (input = Button B, output = green LED)
   * @param dio2 Mode for DIO 2 (input = Button C, output = red LED)
   */
  public OnBoardIO(ChannelMode dio1, ChannelMode dio2) {
    if (dio1 == ChannelMode.INPUT) {
      m_buttonB = new DigitalInput(1);
    } else {
      m_greenLed = new DigitalOutput(1);
    }

    if (dio2 == ChannelMode.INPUT) {
      m_buttonC = new DigitalInput(2);
    } else {
      m_redLed = new DigitalOutput(2);
    }
  }  // end constructor

  /** Gets if the A button is pressed. */
  public boolean getButtonAPressed() {
    return m_buttonA.get();
  }

  /** Gets if the B button is pressed. */
  public boolean getButtonBPressed() {
    if (m_buttonB != null) {
      return m_buttonB.get();
    }

    double currentTime = Timer.getFPGATimestamp();

    if (currentTime > m_nextMessageTime) {
      DriverStation.reportError("Button B was not configured", true);
      m_nextMessageTime = currentTime + MESSAGE_INTERVAL;
    }
    return false;
  }  // end getBB press

  /** Gets if the C button is pressed if it exists */
  public boolean getButtonCPressed() {
    if (m_buttonC != null) {
      return m_buttonC.get();
    }
     // evaluated only if null var ?
    double currentTime = Timer.getFPGATimestamp();
    if (currentTime > m_nextMessageTime) {
      DriverStation.reportError("Button C was not configured", true);
      m_nextMessageTime = currentTime + MESSAGE_INTERVAL;
    }
    return false;
  }  // end get Button C press

  /** Sets the green LED. */
  public void setGreenLed(boolean value) {
    if (m_greenLed != null) {
      m_greenLed.set(value);
    } else {
      double currentTime = Timer.getFPGATimestamp();
      if (currentTime > m_nextMessageTime) {
        DriverStation.reportError("Green LED was not configured", true);
        m_nextMessageTime = currentTime + MESSAGE_INTERVAL;
      }
    }
  }

  /** Sets the red LED if exists [dio2 is output] */
  public void setRedLed(boolean value) {
    if (m_redLed != null) {
      m_redLed.set(value);
    } else {
      double currentTime = Timer.getFPGATimestamp();
      if (currentTime > m_nextMessageTime) {
        DriverStation.reportError("Red LED was not configured", true);
        m_nextMessageTime = currentTime + MESSAGE_INTERVAL;
      }
    }
  }

  /** Sets the yellow LED. */
  public void setYellowLed(boolean value) {
    m_yellowLed.set(value);
  }

  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }
}  // end onBordIO class
