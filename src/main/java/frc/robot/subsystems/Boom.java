// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.Ports;

public class Boom extends ProfiledPIDSubsystem {
  private CANSparkMax m1, m2;
  private MotorControllerGroup mgroup;
  private DutyCycleEncoder absEncoder;
  private AnalogTrigger aLimit, forLimit, revLimit, zeroLimit;

  //private static ArmFeedforward ExtFeedforward = new ArmFeedforward(-.33145, 6.9861, 8.1128);
  //private static ArmFeedforward RetFeedforward = new ArmFeedforward(0.49196, 2.0649, 2.2597);

  private static ProfiledPIDController retPIDController = 
    new ProfiledPIDController(
      6.25,
     0,
     0,//.025,
      // The motion profile constraints
      new TrapezoidProfile.Constraints(3.14, 3.14*.7)
    );

  ShuffleboardTab tab = Shuffleboard.getTab("arm tab");
  double output = 0;
  
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
    super(retPIDController);
    getController().setTolerance(
      .075,
      .12
    );

    m1 = new CANSparkMax(CANConstants.kBoomMotor1Port, MotorType.kBrushless);
    m2 = new CANSparkMax(CANConstants.kBoomMotor2Port, MotorType.kBrushless);
    m1.setSmartCurrentLimit(15);
    m2.setSmartCurrentLimit(15);
    m1.setIdleMode(IdleMode.kBrake);
    m2.setIdleMode(IdleMode.kBrake);

    mgroup = new MotorControllerGroup(m1, m2);
    mgroup.setInverted(true);

    absEncoder = new DutyCycleEncoder(Ports.kArmAbsoluteEncoderPort);

    aLimit = new AnalogTrigger(Ports.kBoomLimitPort);
    aLimit.setLimitsVoltage(4, 4.5);
    revLimit = new AnalogTrigger(Ports.kRevBoomLimitPort);
    revLimit.setLimitsVoltage(4, 4.5);
    forLimit = new AnalogTrigger(Ports.kForBoomLimitPort);
    forLimit.setLimitsVoltage(4, 4.5);
    zeroLimit = new AnalogTrigger(Ports.kZeroBoomLimitPort);
    zeroLimit.setLimitsVoltage(4, 4.5);

    tab.addBoolean("limit", this::getBoomLimit);
    tab.addBoolean("forLimit", forLimit::getTriggerState);
    tab.addBoolean("zeroLimit", zeroLimit::getTriggerState);
    tab.addBoolean("revLimit", revLimit::getTriggerState);
    tab.addDouble("boom amps", ()->{return (m1.getOutputCurrent()+m2.getOutputCurrent())/2;});
    tab.addDouble("measurement", this::getMeasurement);
    tab.addDouble("goal", ()->getController().getGoal().position);
    tab.addDouble("setpoint", ()->getController().getSetpoint().position);
    tab.addDouble("velocity setpoint", ()->getController().getSetpoint().velocity);
    tab.addBoolean("atgoal", this::getAtGoal);
    tab.add("coast mode",runOnce(()->{
      m1.setIdleMode(IdleMode.kCoast);
      m2.setIdleMode(IdleMode.kCoast);
    })).withWidget(BuiltInWidgets.kCommand);
    tab.add("brake mode",runOnce(()->{
      m1.setIdleMode(IdleMode.kBrake);
      m2.setIdleMode(IdleMode.kBrake);
    })).withWidget(BuiltInWidgets.kCommand);
    tab.addDouble("output", ()->output);
  }

  public boolean getBoomLimit(){return aLimit.getTriggerState();}
  
  public CommandBase openClaw(){return runOnce(()->clawSolenoid.set(Value.kForward));}
  public CommandBase closeClaw(){return runOnce(()->clawSolenoid.set(Value.kReverse));}
  public CommandBase extendBoom(){return runOnce(()->boomSolenoid.set(Value.kForward));}
  public CommandBase retractBoom(){return runOnce(()->boomSolenoid.set(Value.kReverse));}

  /**sets goal of pid loop */
  private CommandBase setGoalCommand(double goal){
    return runOnce(()->{setGoal(goal);enable();});
  }
  public CommandBase enableCommand(){return runOnce(()->enable());}
  public CommandBase disableCommand(){return runOnce(()->{disable();mgroup.stopMotor();});}
  public CommandBase resetController(){return runOnce(()->getController().reset(getMeasurement()));}

  public CommandBase setBrakeMode(boolean brake){return runOnce(()->{
    m1.setIdleMode(brake?IdleMode.kBrake:IdleMode.kCoast);
    m2.setIdleMode(brake?IdleMode.kBrake:IdleMode.kCoast);
  });}
  
  /**moves arm to position given (in radia ns) also automatically retracts arm if needed */
  private CommandBase moveArm(double value){
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
      ()->(Math.signum(value)!=Math.signum(getMeasurement())||value==0)
    )
      .andThen(new WaitUntilCommand(this::getAtGoal));
  }
  
  public CommandBase moveToBackTopPosition(){return moveArm(-1.95).andThen(extendBoom());}
  public CommandBase moveToBackMiddlePostion(){return moveArm(-1.7);}
  public CommandBase moveToBackLowPosition(){return moveArm(-.83).andThen(extendBoom());}
  public CommandBase moveToBackIntakePosition(){return moveArm(-.775).andThen(extendBoom());}
  public CommandBase moveToZeroPosition(){return moveArm(0).andThen(disableCommand());}
  public CommandBase moveToFrontIntakePosition(){return moveArm(.755).andThen(extendBoom());}
  public CommandBase moveToFrontGroudPosition(){return moveArm(.8).andThen(extendBoom());}
  public CommandBase moveToFrontMiddlePosition(){return moveArm(1.59).andThen(extendBoom());}
  public CommandBase moveToFHummanPlayerStation(){return moveArm(1.475);}
  public CommandBase moveToRHummanPlayerStation(){return moveArm(-1.55);}

  public boolean getAtGoal(){
    return Math.abs(getController().getGoal().position-getMeasurement())<getController().getPositionTolerance();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    //double appliedVoltage = output+R TetFeedforward.calculate(setpoint.position, setpoint.velocity);
    this.output = output;
    double appliedVoltage = output;
    if(appliedVoltage>0&&forLimit.getTriggerState()){mgroup.stopMotor();disable();return;}
    if(appliedVoltage<0&&revLimit.getTriggerState()){mgroup.stopMotor();disable();return;}
    else{mgroup.setVoltage(appliedVoltage);}
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    //return relencoder.getDistance();
    return 2*Math.PI*(absEncoder.getAbsolutePosition()-.475);
  }

  @Override
  public void periodic() {
      super.periodic();
      //if(getController().getGoal().position==0&&zeroLimit.getTriggerState()){disable();mgroup.stopMotor();}
  }
}
