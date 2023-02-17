// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class alignToGamePiece extends PIDCommand {
  Drivetrain drivetrain; Vision vision;
  /** Creates a new alignToGamePeice. */
  public alignToGamePiece(
    Drivetrain drivetrain, Vision vision, 
    int gamePiece, DoubleSupplier magnitude
  ){
    super(
      // The controller that the command will use
      new PIDController(.2, 0, .01),
      // This should return the measurement
      ()->vision.getResults(gamePiece).getBestTarget().getYaw(),
      // This should return the setpoint (can also be a constant)
      () -> 0,
      // This uses the output
      output -> {
        drivetrain.driveKinematically(magnitude, ()->output);
      });
      this.drivetrain = drivetrain; this.vision = vision;
      addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
