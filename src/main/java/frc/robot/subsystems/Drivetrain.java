// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.RobotConstruction;

public class Drivetrain extends SubsystemBase {
  ShuffleboardTab tab = Shuffleboard.getTab("Telemetry Tab");

  private CANSparkMax l1, l2, r1, r2;
  private static RelativeEncoder lencoder, rencoder;
  private AHRS navx = new AHRS();
  DifferentialDrive drive;

  private static DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(RobotConstruction.kTrackWidth); 
  private DifferentialDriveOdometry odometry;

  private PIDController leftController = new PIDController(0, 0, 0);
  private PIDController rightController = new PIDController(0, 0, 0);
  private SimpleMotorFeedforward leftFeedForward = new SimpleMotorFeedforward(0, 0, 0);
  private SimpleMotorFeedforward rightFeedForward = new SimpleMotorFeedforward(0, 0, 0);

  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    l1 = new CANSparkMax(CANConstants.kL1Port, MotorType.kBrushless);
    l2 = new CANSparkMax(CANConstants.kL2Port, MotorType.kBrushless);
    r1 = new CANSparkMax(CANConstants.kR1Port, MotorType.kBrushless);
    r2 = new CANSparkMax(CANConstants.kR2Port, MotorType.kBrushless);

    l1.setInverted(true);
    r1.setInverted(false);

    l2.follow(l1);
    r2.follow(r1);

    lencoder = l1.getEncoder();
    rencoder = r1.getEncoder();

    drive = new DifferentialDrive(l1, l2);
  }

  public DifferentialDriveKinematics getKinematics(){return kinematics;}

  public static DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(lencoder.getVelocity(), rencoder.getVelocity());
  }

  public static ChassisSpeeds getChassisSpeed(){return kinematics.toChassisSpeeds(getWheelSpeeds());}

  public Pose2d getPose2d(){return new Pose2d();}//TODO change

  public double getLeftDisplacement(){return lencoder.getPosition();}
  public double getRightDisplacement(){return rencoder.getPosition();}

  public double getYaw(){return navx.getAngle();}
  public double getPitch(){return (double)navx.getPitch();}
  public Rotation2d getRotation2d(){return navx.getRotation2d();}
  public double getTurnRate(){return navx.getRate();}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase tempdrive(DoubleSupplier x, DoubleSupplier z) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return this.run(
        () -> {
          drive.arcadeDrive(x.getAsDouble(), z.getAsDouble());
        });
  }

  public void tempDrive2(double x, double z){
    drive.arcadeDrive(x, z);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
