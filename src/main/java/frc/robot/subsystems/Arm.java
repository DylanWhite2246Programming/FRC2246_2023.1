// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.Ports;

public class Arm extends TrapezoidProfileSubsystem {
  //private CANSparkMax m1;
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
  
  /** Creates a new Arm. */
  public Arm() {
    //m1.getPIDController().setFeedbackDevice((MotorFeedbackSensor)new DutyCycleEncoder(0));
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(0, 0),
        // The initial position of the mechanism
        0);
  }

  public CommandBase openClaw(){return runOnce(()->clawSolenoid.set(Value.kForward));}
  public CommandBase closeClaw(){return runOnce(()->clawSolenoid.set(Value.kReverse));}
  public CommandBase extendBoom(){return runOnce(()->boomSolenoid.set(Value.kForward));}
  public CommandBase retractBoom(){return runOnce(()->boomSolenoid.set(Value.kReverse));}

  @Override
  protected void useState(TrapezoidProfile.State state) {
    // Use the computed profile state here.
  }
}
