// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
    //ANALOG INPUT
    public static final int kBoomLimitPort = 0;
    public static final int kForBoomLimitPort = 1;
    public static final int kZeroBoomLimitPort = 2;
    public static final int kRevBoomLimitPort = 3;
    public static final int kLeftBrakeLimitPort = 4;
    public static final int kRightBrakeLimitPort = 5;
    //SOLENOIDS
    public static final int kClawForwardPort = 15;
    public static final int kClawReversePort = 14;
    public static final int kBoomForwardPort = 12;
    public static final int kBoomReversePort = 13;
    public static final int kBrakeForwardPort = 10;
    public static final int kBrakeReversePort = 11;
  }
  public static class AutonControllers{
    public static final PIDController leftPositionController = 
      new PIDController(0, 0, 0);
    public static final PIDController rightPositionController = 
      new PIDController(0, 0, 0);
    public static final PIDController turnController = 
      new PIDController(.15, 0, 0);
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class RobotConstruction{
    public static final double kTrackWidth = Units.inchesToMeters(22+(7/16));
    public static final double kEncoderPositionConverionRate = 1/(Units.inchesToMeters(6)*Math.PI*10.71);//TODO verify these values
    public static final double kEncoderVelocityConverionRate = kEncoderPositionConverionRate/60; //TODO verify these values
    public static final Transform3d robotToCam = new Transform3d(new Translation3d(), new Rotation3d()); //TODO put value here
  }
}
