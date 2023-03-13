// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AutonControllers;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.RobotConstruction;

public class Drivetrain extends SubsystemBase {
  ShuffleboardTab tab = Shuffleboard.getTab("Telemetry Tab");
  private Vision vision;

  private double xScalar = 3.75, zScalar = 3.14;
  private CANSparkMax l1, l2, r1, r2;
  private CANSparkMax[] motorArray;
  private MotorControllerGroup lMotorGroup, rMotorGroup;
  private DifferentialDrive drive;

  private static DoubleSolenoid brakeSolenoid = new DoubleSolenoid(
    CANConstants.kPHPort, 
    PneumaticsModuleType.REVPH, 
    Ports.kBrakeForwardPort, 
    Ports.kBrakeReversePort
  );
  private static AnalogTrigger leftLimit, rightLimit;

  private static RelativeEncoder l1encoder, l2encoder, r1encoder, r2encoder;
  private RelativeEncoder[] reArray;
  
  private AHRS navx = new AHRS(Port.kMXP);

  private static DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(RobotConstruction.kTrackWidth); 
  private DifferentialDriveOdometry odometry;

  Constraints constraints = new Constraints(4, .000005);
  private PIDController leftVelocityController  = new PIDController(.060762, 0, 0);
  private PIDController rightVelocityController = new PIDController(0.060762, 0, 0);
  //old ks 12236
  private SimpleMotorFeedforward leftFeedForward = new SimpleMotorFeedforward(.053405, 2.698, .32867);
  private SimpleMotorFeedforward rightFeedForward = new SimpleMotorFeedforward(.053405, 2.698, .32857);

  /** Creates a new ExampleSubsystem. */
  public Drivetrain(Vision cam) {
    l1 = new CANSparkMax(CANConstants.kL1Port, MotorType.kBrushless);
    l2 = new CANSparkMax(CANConstants.kL2Port, MotorType.kBrushless);
    r1 = new CANSparkMax(CANConstants.kR1Port, MotorType.kBrushless);
    r2 = new CANSparkMax(CANConstants.kR2Port, MotorType.kBrushless);

    motorArray = new CANSparkMax[]{l1,l2,r1,r2};

    for(CANSparkMax i : motorArray){
      i.setOpenLoopRampRate(.75);
    }

    lMotorGroup = new MotorControllerGroup(l1, l2);
    rMotorGroup = new MotorControllerGroup(r1, r2);

    lMotorGroup.setInverted(false);
    rMotorGroup.setInverted(false);

    drive = new DifferentialDrive(lMotorGroup, rMotorGroup);
    drive.setSafetyEnabled(false);

    setIdleMode(IdleMode.kBrake);

    l1encoder = l1.getEncoder();
    l2encoder = l2.getEncoder();
    r1encoder = r1.getEncoder();
    r2encoder = r2.getEncoder();
    reArray = new RelativeEncoder[]{l1encoder,l2encoder,r1encoder,r2encoder};
    
    for(RelativeEncoder i : reArray){
      i.setPositionConversionFactor(RobotConstruction.kEncoderPositionConverionRate);
      i.setVelocityConversionFactor(RobotConstruction.kEncoderVelocityConverionRate);
      i.setPosition(0);
    }

    leftLimit = new AnalogTrigger(Ports.kLeftBrakeLimitPort);
    leftLimit.setLimitsVoltage(4, 4.5);
    rightLimit = new AnalogTrigger(Ports.kRightBrakeLimitPort);
    rightLimit.setLimitsVoltage(4, 4.5);

    brakeSolenoid.set(Value.kReverse);

    vision = cam;

    odometry = new DifferentialDriveOdometry(getRotation2d(), getLeftDisplacement(), getRightDisplacement());

    tab.add(drive).withWidget(BuiltInWidgets.kDifferentialDrive);
    tab.addDouble("pitch",this::getPitch);
    tab.add("coast mode", runOnce(()->setIdleMode(IdleMode.kCoast)));
    tab.add("brake mode", runOnce(()->setIdleMode(IdleMode.kBrake)));
    tab.add("zero pose", runOnce(()->resetPose()));
    tab.add("engageBrake", this.engageBrake());
    tab.add("disengageBrake", this.disengageBrake());
    tab.addBoolean("brake status", this::getBrakeStatus);
    tab.addDouble("x", ()->getPose2d().getX());
  }

  public DifferentialDriveKinematics getKinematics(){return kinematics;}

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(l1encoder.getVelocity(), r1encoder.getVelocity());
  }

  public ChassisSpeeds getChassisSpeed(){return kinematics.toChassisSpeeds(getWheelSpeeds());}

  public Pose2d getPose2d(){return odometry.getPoseMeters();}
  public void resetPose(){
    odometry.resetPosition(
      getRotation2d(), 
      getLeftDisplacement(), 
      getRightDisplacement(), 
      new Pose2d(0, 0, new Rotation2d())
    );
  }

  public double getLeftDisplacement(){return l1encoder.getPosition();}
  public double getRightDisplacement(){return r1encoder.getPosition();}

  public double getLeftVelocity(){return l1encoder.getVelocity();}
  public double getRightVelocity(){return r1encoder.getVelocity();}

  public double getYaw(){return navx.getAngle();}
  public double getPitch(){return (double)navx.getRoll()-1;}
  double lastPitch=0, lastTimeStamp = 0;
  ArrayList<Double> lastValues = new ArrayList<>();
  public double getPitchRate(){
    double currentTime = RobotController.getFPGATime();
    if(currentTime-lastTimeStamp>.3){lastValues.clear();}
    do{
      currentTime = RobotController.getFPGATime()/1000000;
      var d = (getPitch()-lastPitch)/(currentTime-lastTimeStamp); 
      lastValues.add(d);
      lastPitch=getPitch(); lastTimeStamp=currentTime;
    }while(lastValues.size()<10);
    if(lastValues.size()>10){
      lastValues=(ArrayList<Double>)lastValues.subList(0, 10);
    }
    //10 represents the number of vlaues to keep
    double sum = 0;
    for(Object i: lastValues.toArray()){sum+=(double)i;}
    return sum/lastValues.size();
  }
  public Rotation2d getRotation2d(){return navx.getRotation2d();}
  public double getTurnRate(){return navx.getRate();}
  public CommandBase calibrateNavx(){
    return runOnce(()->navx.calibrate())
      .andThen(new WaitUntilCommand(()->!navx.isCalibrating()))
      .andThen(runOnce(()->navx.reset()));
  }

  /**when true brake is not engaged*/
  public boolean getBrakeStatus(){return rightLimit.getTriggerState()&&leftLimit.getTriggerState();}
  public CommandBase engageBrake(){return runOnce(()->brakeSolenoid.set(Value.kForward));}
  public CommandBase disengageBrake(){return runOnce(()->brakeSolenoid.set(Value.kReverse));}

  public CommandBase driveKinematically(DoubleSupplier x, DoubleSupplier z, BooleanSupplier override){
    return this.runEnd(()->{
      DifferentialDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(
        new ChassisSpeeds(
          x.getAsDouble()*xScalar, 0, 
          z.getAsDouble()*zScalar
        ));
      if(!(getBrakeStatus()||override.getAsBoolean())){driveVolts(0, 0);return;}
      if((x.getAsDouble()!=0||z.getAsDouble()!=0))
      driveVolts( //this call also feeds drivetrain
        leftFeedForward.calculate(speeds.leftMetersPerSecond)
          +leftVelocityController.calculate(getWheelSpeeds().leftMetersPerSecond, speeds.leftMetersPerSecond), 
        rightFeedForward.calculate(speeds.rightMetersPerSecond)
          +rightVelocityController.calculate(getWheelSpeeds().rightMetersPerSecond, speeds.rightMetersPerSecond)
      );
      else{driveVolts(0, 0);}
    },()->driveVolts(0, 0));
  }

  public CommandBase drivePorpotionaly(DoubleSupplier x, DoubleSupplier z) {
    return this.run(
        () -> {
          drive.arcadeDrive(x.getAsDouble(), z.getAsDouble());
        });
  }

  public CommandBase allignToGamePiece(DoubleSupplier x){
    return drivePorpotionaly(
      x, 
      ()->{
        if(vision.getResults(1).hasTargets()&&(Math.abs(vision.getResults(1).getBestTarget().getYaw())>.05))
        return AutonControllers.turnController.calculate((vision.getResults(1)).getBestTarget().getYaw(), 0);
        else return 0;
      }
      );
  }
    
  public void driveVolts(double lVolt, double rVolt){
    lMotorGroup.setVoltage(lVolt);
    rMotorGroup.setVoltage(rVolt);
    drive.feed();
  }

  public CommandBase stopCommand(){
    return runOnce(()->driveVolts(0, 0));
  }

  public void setIdleMode(IdleMode mode){
    for(CANSparkMax i : motorArray){
      i.setIdleMode(mode);
    }
  }

  public CommandBase setBrakeMode(){return runOnce(()->setIdleMode(IdleMode.kBrake));}
  public CommandBase setCoastMode(){return runOnce(()->setIdleMode(IdleMode.kCoast));}


  @Override
  public void periodic() {
    odometry.update(getRotation2d(), getLeftDisplacement(), getRightDisplacement());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
