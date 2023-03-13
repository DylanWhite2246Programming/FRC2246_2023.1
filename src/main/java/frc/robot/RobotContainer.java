// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Team2246.Drivestation;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Boom;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PowerAndPneumatics;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
  ShuffleboardTab tab = Shuffleboard.getTab("main");
  SendableChooser<CommandBase> chooser = new SendableChooser<CommandBase>();

  // The robot's subsystems and commands are defined here...
  private final Vision cam = new Vision();
  private final Boom boom = new Boom();
  private final Drivetrain drivetrain = new Drivetrain(cam);
  private final PowerAndPneumatics pp = new PowerAndPneumatics();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain.resetPose();
    drivetrain.setDefaultCommand(drivetrain.driveKinematically(drivestation::getLeftY, drivestation::getRightX, ()->true));
    // Configure the trigger bindings
    configureBindings();
    //chooser.addOption("onegamepiece", Autos.oneGameAndTaxi(drivetrain, boom));
    chooser.setDefaultOption("onegamepiece", Autos.oneGameAndTaxi(drivetrain, boom));
    //chooser.addOption("autoleveltest", 
    //  drivetrain.run(()->drivetrain.driveVolts(-1.7, -1.7))
    //    .until(()->{return Math.abs(drivetrain.getPitch())>14;})
    //    .withTimeout(5)
    //  .andThen(new AutoLevel(drivetrain))
    //);
    chooser.addOption("autolevel?", Autos.oneGameAndLevel(drivetrain, boom));
    chooser.addOption("oneGameCable", Autos.oneGameCable(drivetrain, boom));
    chooser.addOption("non", null);
    tab.add("sendable", chooser).withSize(2, 1);
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
    //new Trigger(()->drivestation.getRightPov()==90).whileTrue(new alignToGamePiece(drivetrain, cam, 1, drivestation::getLeftY));
    //new Trigger(()->drivestation.getRightPov()==90).whileTrue(drivetrain.allignToGamePiece(drivestation::getLeftY));
    //new Trigger(()->drivestation.getRightPov()==180).whileTrue(new alignToGamePiece(drivetrain, cam, 2, ()->0));
    drivestation.ls1().onTrue(boom.moveToZeroPosition());
    drivestation.ls3().onTrue(boom.moveToFrontIntakePosition());
    drivestation.ls2().onTrue(boom.moveToBackIntakePosition());
    drivestation.ls4().onTrue(boom.openClaw());
    drivestation.ls5().onTrue(boom.closeClaw());

    drivestation.rs2().whileTrue(drivetrain.drivePorpotionaly(drivestation::getLeftY, drivestation::getLeftX));
    drivestation.rs3().onTrue(boom.moveToFHummanPlayerStation());
    drivestation.rs4().onTrue(boom.moveToRHummanPlayerStation());

    drivestation.s13().whileTrue(pp.turnOnCompressorCommand())
      .whileFalse(pp.turnOffCompressorCommand());
    drivestation.getHandBrake().onTrue(drivetrain.engageBrake())
      .onFalse(drivetrain.disengageBrake());
    
    drivestation.b00().onTrue(boom.openClaw());
    drivestation.b01().onTrue(boom.closeClaw());
    drivestation.b10().onTrue(boom.disableCommand());
    drivestation.b11().onTrue(boom.moveToFHummanPlayerStation());
    drivestation.b20().onTrue(boom.extendBoom());
    drivestation.b21().onTrue(boom.retractBoom());
    drivestation.b23().onTrue(boom.moveToBackTopPosition());
    drivestation.b13().onTrue(boom.moveToBackMiddlePostion());
    drivestation.b03().onTrue(boom.moveToBackLowPosition());
    drivestation.b02().onTrue(boom.moveToZeroPosition());
    drivestation.b12().onTrue(boom.moveToFrontQuePosition());
    drivestation.b22().onTrue(boom.moveToFrontMiddlePosition());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return chooser.getSelected().beforeStarting(()->drivetrain.resetPose(),drivetrain);
  }

  public void periodic(){}
  public void teleopDisabledInit(){}
  public void disabledInit(){boom.disable();}
}
