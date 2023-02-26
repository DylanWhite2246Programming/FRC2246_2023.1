// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Team2246.Drivestation;
import frc.robot.commands.alignToGamePiece;
import frc.robot.subsystems.Boom;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PowerAndPneumatics;
import frc.robot.subsystems.Vision;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivestation drivestation = new Drivestation(0, 1, 2, 3);

  // The robot's subsystems and commands are defined here...
  private final Vision cam = new Vision();
  private final Boom boom = new Boom();
  private final Drivetrain drivetrain = new Drivetrain(cam);
  private final PowerAndPneumatics pp = new PowerAndPneumatics();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain.setDefaultCommand(drivetrain.driveKinematically(drivestation::getLeftY, drivestation::getRightX, ()->true));
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    BooleanSupplier boomLimitOveride = drivestation.getBoomLimitOveride();
    new Trigger(()->drivestation.getRightPov()==90).whileTrue(new alignToGamePiece(drivetrain, cam, 1, ()->0));
    new Trigger(()->drivestation.getRightPov()==180).whileTrue(new alignToGamePiece(drivetrain, cam, 2, ()->0));
    drivestation.ls1().onTrue(boom.moveToZeroPosition(boomLimitOveride.getAsBoolean()));
    drivestation.ls2().onTrue(boom.moveToFrontIntakePosition(boomLimitOveride.getAsBoolean()));
    drivestation.ls3().onTrue(boom.moveToBackIntakePosition(boomLimitOveride.getAsBoolean()));
    drivestation.ls4().onTrue(boom.openClaw());
    drivestation.ls5().onTrue(boom.closeClaw());

    drivestation.s13().whileTrue(pp.turnOnCompressorCommand())
      .whileFalse(pp.turnOffCompressorCommand());
    drivestation.getHandBrake().onTrue(drivetrain.engageBrake())
      .onFalse(drivetrain.disengageBrake());
    
    drivestation.b00().onTrue(boom.openClaw());
    drivestation.b01().onTrue(boom.closeClaw());
    drivestation.b10().onTrue(boom.disableCommand());
    drivestation.b20().onTrue(boom.extendBoom());
    drivestation.b21().onTrue(boom.retractBoom());
    drivestation.b23().onTrue(boom.moveToBackTopPosition(boomLimitOveride.getAsBoolean()));
    drivestation.b13().onTrue(boom.moveToBackMiddlePostion(boomLimitOveride.getAsBoolean()));
    drivestation.b03().onTrue(boom.moveToBackLowPosition(boomLimitOveride.getAsBoolean()));
    drivestation.b02().onTrue(boom.moveToZeroPosition(boomLimitOveride.getAsBoolean()));
    drivestation.b12().onTrue(boom.moveToFrontGroudPosition(boomLimitOveride.getAsBoolean()));
    drivestation.b22().onTrue(boom.moveToFrontMiddlePosition(boomLimitOveride.getAsBoolean()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }

  public void periodic(){}
}
