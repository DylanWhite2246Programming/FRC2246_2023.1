// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.Ports;

public class Boom extends ProfiledPIDSubsystem {
  private CANSparkMax m1, m2; 
  private MotorControllerGroup mgroup;
  private Encoder relencoder;
  private DigitalInput aLimit;

  private ArmFeedforward ExtFeedforward = new ArmFeedforward(-.33145, 6.9861, 8.1128);
  private ArmFeedforward RetBackFeedforward = new ArmFeedforward(6.295, 2.3063, 6.0698);

  ShuffleboardTab tab = Shuffleboard.getTab("arm tab");
  
  private DoubleSolenoid boomSolenoid 
    = new DoubleSolenoid(
      CANConstants.kPHPort, 
      PneumaticsModuleType.REVPH, 
      Ports.kBoomForwardPort, 
      Ports.kBoomReversePort
    );

  private DoubleSolenoid clawSolenoid
    = new DoubleSolenoid(
      CANConstants.kPHPort,
      PneumaticsModuleType.REVPH, 
      Ports.kClawForwardPort, 
      Ports.kClawReversePort
    );

  /** Creates a new Boom. */
  public Boom() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            6.295/2,
           0,
              10.121*.9,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(3.14/8, 3.14/2))
      );
    //getController().setTolerance(
    //  0.7854,
    //  0.36045
    //);

    m1 = new CANSparkMax(CANConstants.kBoomMotor1Port, MotorType.kBrushless);
    m2 = new CANSparkMax(CANConstants.kBoomMotor2Port, MotorType.kBrushless);
    m1.setIdleMode(IdleMode.kBrake);
    m2.setIdleMode(IdleMode.kBrake);

    mgroup = new MotorControllerGroup(m1, m2);
    mgroup.setInverted(true);

    relencoder = new Encoder(2, 1, true);
    relencoder.setDistancePerPulse(2*Math.PI/4096);
    relencoder.setSamplesToAverage(7);

    aLimit = new DigitalInput(Ports.kBoomLimitSwitchPortA);

    tab.addDouble("measurement", this::getMeasurement);
    tab.addBoolean("limit", this::getBoomLimit);
    tab.add("reset encoder",runOnce(()->relencoder.reset()))
      .withWidget(BuiltInWidgets.kCommand);
    tab.add("coast mode",runOnce(()->{
      m1.setIdleMode(IdleMode.kCoast);
      m2.setIdleMode(IdleMode.kCoast);
    })).withWidget(BuiltInWidgets.kCommand);
    tab.add("brake mode",runOnce(()->{
      m1.setIdleMode(IdleMode.kBrake);
      m2.setIdleMode(IdleMode.kBrake);
    })).withWidget(BuiltInWidgets.kCommand);
    tab.addBoolean("atgoal", ()->getController().atGoal());
    tab.addDouble("boom amps", ()->{return (m1.getOutputCurrent()+m2.getOutputCurrent())/2;});
  }

  public boolean getBoomLimit(){return !aLimit.get();}
  
  public CommandBase openClaw(){return runOnce(()->clawSolenoid.set(Value.kForward));}
  public CommandBase closeClaw(){return runOnce(()->clawSolenoid.set(Value.kReverse));}
  public CommandBase extendBoom(){return runOnce(()->boomSolenoid.set(Value.kForward));}
  public CommandBase retractBoom(){return runOnce(()->boomSolenoid.set(Value.kReverse));}

  /**sets goal of pid loop */
  private CommandBase setGoalCommand(double goal){
    return new ParallelDeadlineGroup(
      new WaitUntilCommand(()->getController().atGoal()),
      runOnce(()->{setGoal(goal);enable();})
    );
  }
  public CommandBase enableCommand(){return runOnce(()->enable());}
  public CommandBase disableCommand(){return runOnce(()->{disable();mgroup.stopMotor();});}

  public CommandBase setBrakeMode(boolean brake){return runOnce(()->{
    m1.setIdleMode(brake?IdleMode.kBrake:IdleMode.kCoast);
    m2.setIdleMode(brake?IdleMode.kBrake:IdleMode.kCoast);
  });}
  
  /**moves arm to position given (in radia ns) also automatically retracts arm if needed */
  private CommandBase moveArm(double value, boolean limOveride){
    return new ConditionalCommand(
      //retract arm and wait for it to reach limit and then move arm
      new SequentialCommandGroup(
        retractBoom(),
        new WaitUntilCommand(this::getBoomLimit),
        setGoalCommand(value)
      ),
      //if collision will not happen move arm
      setGoalCommand(value), 
      //when the goal and curent position are on differnt sides of the robot the arm must be retracted
      ()->(Math.signum(value)!=Math.signum(this.getMeasurement())||value==0)&&!limOveride
    ).andThen(disableCommand());
  }
  
  public CommandBase moveToBackTopPosition(Boolean limOveride){return moveArm(0,limOveride);}
  public CommandBase moveToBackMiddlePostion(Boolean limOveride){return moveArm(0,limOveride);}
  public CommandBase moveToBackLowPosition(Boolean limOveride){return moveArm(0,limOveride);}
  public CommandBase moveToBackIntakePosition(boolean limOveride){return moveArm(0,limOveride);}
  public CommandBase moveToZeroPosition(Boolean limOveride){return moveArm(-.5,limOveride);}
  public CommandBase moveToFrontIntakePosition(Boolean limOveride){return moveArm(0,limOveride);}
  public CommandBase moveToFrontGroudPosition(Boolean limOveride){return moveArm(0,limOveride);}
  public CommandBase moveToFrontMiddlePosition(Boolean limOveride){return moveArm(0,limOveride);}

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    //if(getBoomLimit()){
    //}else{
    //  mgroup.setVoltage(output+ExtFeedforward.calculate(setpoint.position, setpoint.velocity));
    //}
    //mgroup.setVoltage(output+RetFeedforward.calculate(setpoint.position, setpoint.velocity));
    mgroup.setVoltage(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return relencoder.getDistance();
  }

  @Override
  public void periodic() {
      // TODO Auto-generated method stub
      super.periodic();
  }
}
