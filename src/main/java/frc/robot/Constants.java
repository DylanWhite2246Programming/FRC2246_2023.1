// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int[] pressure = new int[]{65,85};
  public static class CANConstants{
    public static final int kL1Port = 1;
    public static final int kL2Port = 2;
    public static final int kR1Port = 3;
    public static final int kR2Port = 4;
    public static final int kBoomMotor1Port = 5; 
    public static final int kBoomMotor2Port = 6; 
    public static final int kPDHPort = 7;
    public static final int kPHPort = 8;
  }
  public static class Ports{
    //PWM
    public static final int kLedPort = 0; //TODO find leds
    //DIO
    public static final int kArmAbsoluteEncoderPort = 0;
    public static final int kArmRelativeAEncoderPort = 1;
    public static final int kArmRelativeBEncoderPort = 2;
    public static final int kBoomLimitSwitchPortA = 3;
    public static final int kBoomLimitSwitchPortB = 4;
    //SOLENOIDS
    public static final int kClawForwardPort = 1;
    public static final int kClawReversePort = 2;
    public static final int kBoomForwardPort = 3;
    public static final int kBoomReversePort = 4;
      //RESERVED FOR TWO STAGED CLAW
    public static final int kBrakeForwardPort = 7;
    public static final int kBrakeReversePort = 8;
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class RobotConstruction{
    public static final double kTrackWidth = Units.inchesToMeters(0);
    public static final double kEncoderPositionConverionRate = 1/(Units.inchesToMeters(6)*Math.PI*10.71);
    public static final double kEncoderVelocityConverionRate = kEncoderPositionConverionRate/60;
  }
}
