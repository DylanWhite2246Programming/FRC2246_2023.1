// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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


public class arm2 extends SubsystemBase {
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
  /** Creates a new arm2. */
  public arm2() {
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

    //tab.addDouble("measurement", this::getMeasurement);
    //tab.addBoolean("limit", this::getBoomLimit);
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
    //tab.addBoolean("atgoal", ()->getController().atGoal());
    tab.addDouble("boom amps", ()->{return (m1.getOutputCurrent()+m2.getOutputCurrent())/2;});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
