// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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
  private DutyCycleEncoder absencoder;
  private DigitalInput aLimit, bLimit;
  private boolean limitOveride = false;

  private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0);
  
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
            0,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0, 0)));

    m1 = new CANSparkMax(CANConstants.kBoomMotor1Port, MotorType.kBrushless);
    m2 = new CANSparkMax(CANConstants.kBoomMotor2Port, MotorType.kBrushless);
    mgroup = new MotorControllerGroup(m1, m2);

    absencoder = new DutyCycleEncoder(Ports.kArmAbsoluteEncoderPort);

    mgroup.setInverted(false);

    aLimit = new DigitalInput(Ports.kBoomLimitSwitchPortA);
    bLimit = new DigitalInput(Ports.kBoomLimitSwitchPortB);
  }

  public boolean getBoomLimit(){return aLimit.get()/*|| bLimit.get()*/;}
  
  public CommandBase setOverride(boolean value){return runOnce(()->this.limitOveride=value);}

  public CommandBase openClaw(){return runOnce(()->clawSolenoid.set(Value.kForward));}
  public CommandBase closeClaw(){return runOnce(()->clawSolenoid.set(Value.kReverse));}
  public CommandBase extendBoom(){return runOnce(()->boomSolenoid.set(Value.kForward));}
  public CommandBase retractBoom(){return runOnce(()->boomSolenoid.set(Value.kReverse));}

  /**sets goal of pid loop */
  private CommandBase setGoalCommand(double goal){return runOnce(()->{setGoal(goal);enable();});}

  /**moves arm to position given (in radians) also automatically retracts arm if needed */
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
      ()->(Math.signum(value)!=Math.signum(this.getMeasurement())||value==0)&&!limitOveride
    );
  }
  
  public CommandBase moveToBackTopPosition(){return moveArm(0).andThen(extendBoom());}
  public CommandBase moveToBackMiddlePostion(){return moveArm(0).andThen(retractBoom());}
  public CommandBase moveToBackLowPosition(){return moveArm(0).andThen(extendBoom());}
  public CommandBase moveToZeroPosition(){return moveArm(0).andThen(()->disable());}
  public CommandBase moveToIntakePosition(){return moveArm(0).andThen(extendBoom());}
  public CommandBase moveToFrontGroudPosition(){return moveArm(0).andThen(extendBoom());}
  public CommandBase moveToFrontMiddlePosition(){return moveArm(0).andThen(extendBoom());}
  

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    mgroup.setVoltage(output+feedforward.calculate(setpoint.position, setpoint.velocity));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return absencoder.get()-0;
  }
}
