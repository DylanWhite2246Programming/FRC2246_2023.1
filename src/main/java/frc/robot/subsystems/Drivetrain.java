// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonPipelineResult;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.RobotConstruction;

public class Drivetrain extends SubsystemBase {
  ShuffleboardTab tab = Shuffleboard.getTab("Telemetry Tab");
  private Vision vision;

  private double xScalar = 6.75, zScalar = 7;
  private CANSparkMax l1, l2, r1, r2;
  private CANSparkMax[] motorArray;
  private MotorControllerGroup lMotorGroup, rMotorGroup;
  private DifferentialDrive drive;

  private static RelativeEncoder l1encoder, l2encoder, r1encoder, r2encoder;
  private RelativeEncoder[] reArray;
  
  private AHRS navx = new AHRS();

  private static DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(RobotConstruction.kTrackWidth); 
  private DifferentialDriveOdometry odometry;

  Constraints constraints = new Constraints(8, 1);
  private ProfiledPIDController leftVelocityController  = new ProfiledPIDController(.060762, 0, 0, constraints);
  private ProfiledPIDController rightVelocityController = new ProfiledPIDController(0.060762, 0, 0, constraints);
  //old ks 12236
  private SimpleMotorFeedforward leftFeedForward = new SimpleMotorFeedforward(.053405, 2.698, .32867);
  private SimpleMotorFeedforward rightFeedForward = new SimpleMotorFeedforward(.053405, 2.698, .32857);

  /** Creates a new ExampleSubsystem. */
  public Drivetrain(Vision cam) {
    l1 = new CANSparkMax(CANConstants.kL1Port, MotorType.kBrushless);
    l2 = new CANSparkMax(CANConstants.kL2Port, MotorType.kBrushless);
    r1 = new CANSparkMax(CANConstants.kR1Port, MotorType.kBrushless);
    r2 = new CANSparkMax(CANConstants.kR2Port, MotorType.kBrushless);

    lMotorGroup = new MotorControllerGroup(l1, l2);
    rMotorGroup = new MotorControllerGroup(r1, r2);

    motorArray = new CANSparkMax[]{l1,l2,r1,r2};

    vision = cam;

    lMotorGroup.setInverted(false);
    rMotorGroup.setInverted(false);

    setIdleMode(IdleMode.kCoast);

    drive = new DifferentialDrive(lMotorGroup, rMotorGroup);
    drive.setSafetyEnabled(false);

    l1encoder = l1.getEncoder();
    l2encoder = l2.getEncoder();
    r1encoder = r1.getEncoder();
    r2encoder = r2.getEncoder();
    reArray = new RelativeEncoder[]{l1encoder,l2encoder,r1encoder,r2encoder};

    for(RelativeEncoder i : reArray){
      i.setPositionConversionFactor(RobotConstruction.kEncoderPositionConverionRate);
      i.setVelocityConversionFactor(RobotConstruction.kEncoderVelocityConverionRate);
    }

    odometry = new DifferentialDriveOdometry(getRotation2d(), getLeftDisplacement(), getRightDisplacement());

    tab.add(drive).withWidget(BuiltInWidgets.kDifferentialDrive);
  }

  public DifferentialDriveKinematics getKinematics(){return kinematics;}

  public static DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(l1encoder.getVelocity(), r1encoder.getVelocity());
  }

  public static ChassisSpeeds getChassisSpeed(){return kinematics.toChassisSpeeds(getWheelSpeeds());}

  public Pose2d getPose2d(){return odometry.getPoseMeters();}//TODO change

  public double getLeftDisplacement(){return l1encoder.getPosition();}
  public double getRightDisplacement(){return r1encoder.getPosition();}

  public double getLeftVelocity(){return l1encoder.getVelocity();}
  public double getRightVelocity(){return r1encoder.getVelocity();}

  public double getYaw(){return navx.getAngle();}
  public double getPitch(){return (double)navx.getPitch();}
  public Rotation2d getRotation2d(){return navx.getRotation2d();}
  public double getTurnRate(){return navx.getRate();}

  public CommandBase driveKinematically(DoubleSupplier x, DoubleSupplier z){
    return this.run(()->{
      DifferentialDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(new ChassisSpeeds(x.getAsDouble()*xScalar, 0, z.getAsDouble()*zScalar));
      driveVolts( //this call also feeds drivetrain
        leftFeedForward.calculate(speeds.leftMetersPerSecond)
          +leftVelocityController.calculate(getWheelSpeeds().leftMetersPerSecond, speeds.leftMetersPerSecond), 
        rightFeedForward.calculate(speeds.rightMetersPerSecond)
          +rightVelocityController.calculate(getWheelSpeeds().rightMetersPerSecond, speeds.rightMetersPerSecond)
      );
    });
  }

  public CommandBase drivePorpotionaly(DoubleSupplier x, DoubleSupplier z) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return this.run(
        () -> {
          drive.arcadeDrive(x.getAsDouble(), z.getAsDouble());
          drive.feed();
        });
  }

  public void setIdleMode(IdleMode mode){
    for(CANSparkMax i:motorArray){
      i.setIdleMode(mode);
    }
  }

  public void driveVolts(double lVolt, double rVolt){
    lMotorGroup.setVoltage(lVolt);
    rMotorGroup.setVoltage(rVolt);
    drive.feed();
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
