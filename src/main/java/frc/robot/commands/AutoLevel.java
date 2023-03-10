// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoLevel extends CommandBase {
  Drivetrain drivetrain;
  Timer timer = new Timer();
  private double error, currentAngle, drivePower;
  private final double levelValue = 0, tollerance = 1;

  /** Creates a new AutoLever. */
  public AutoLevel(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }
  
  private boolean isLevel(){return Math.abs(currentAngle)<tollerance;}

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){}//drivetrain.driveKinematically(()->{return drivePower;}, ()->0, ()->false).schedule();;}
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(currentAngle)<8.5){drivetrain.driveVolts(0, 0);;return;}
    //set cuttent angle set to the pitch of the robot
    currentAngle=drivetrain.getPitch();
    //currentAngle = Units.radiansToDegrees(drivetrain.getPitch());
    //calculate error
    error = isLevel()?0:levelValue-currentAngle;
    //calculate the amount of power need.
    //second value is top speed first value is kP
    drivePower = .5*error+.1; 
    if(Math.abs(drivePower)>1){
      drivetrain.driveVolts(Math.signum(drivePower)*1, Math.signum(drivePower)*1);
    }
    else drivetrain.driveVolts(drivePower, drivePower);
    //if angle of table is lower than 2 degrees 
    //2.5degrees is what is required to score
    if(isLevel()){timer.start();}
    else{timer.stop();timer.reset();}//if level is not level rest timer
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopCommand().schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //when the scale is level for more than 1.5 seconds command ends
    System.out.println(timer.get());
    return false;
    //return timer.get()>=1.5;
  }
}
