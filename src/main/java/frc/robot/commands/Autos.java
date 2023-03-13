// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutonControllers;
import frc.robot.subsystems.Boom;
import frc.robot.subsystems.Drivetrain;

public final class Autos {
  ///** Example static factory for an autonomous command. */
  //public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  //  return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  //}

  private static CommandBase ramsetGenerator(Drivetrain drivetrain, Trajectory trajectory){
    return new RamseteCommand(
      trajectory, 
      drivetrain::getPose2d, 
      AutonControllers.ramsetController, 
      drivetrain.getKinematics(), 
      drivetrain::driveVolts,
      drivetrain
    );
    //return new RamseteCommand(
    //  trajectory, 
    //  drivetrain::getPose2d, 
    //  AutonControllers.ramsetController, 
    //  new SimpleMotorFeedforward(0.096056, 6.8672, 0.48042), 
    //  drivetrain.getKinematics(), 
    //  drivetrain::getWheelSpeeds, 
    //  new PIDController(23.206, 0, 1.5709), 
    //  new PIDController(23.248, 0, 2.508), 
    //  drivetrain::driveVolts, 
    //  drivetrain
    //);
  }

  public static CommandBase oneGameAndTaxi(Drivetrain drive, Boom boom){
    drive.resetPose(); double trajectoryToleranceFuckery = .2;
    Pose2d initPose = drive.getPose2d();
    Pose2d placementPose = new Pose2d(
      initPose.getX()-.3-trajectoryToleranceFuckery, 
      initPose.getY(), 
      initPose.getRotation()
    );
    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
      initPose,  
      List.of(), 
      placementPose, 
      AutonControllers.revTrajectoryConfig
    );
    Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
      placementPose, 
      List.of(), 
      new Pose2d(initPose.getX()+4+trajectoryToleranceFuckery, initPose.getY(), initPose.getRotation()), 
      AutonControllers.trajectoryConfig
    );
    return new SequentialCommandGroup(
      boom.closeClaw(),
      boom.resetController(),
      boom.moveToBackTopPosition(),
      ramsetGenerator(drive, trajectory1),
      drive.stopCommand(),
      boom.openClaw(),
      new WaitCommand(1),
      ramsetGenerator(drive, trajectory2).alongWith(boom.moveToZeroPosition()),
      drive.stopCommand()
    );
  }

  public static CommandBase oneGameCable(Drivetrain drive, Boom boom){
    drive.resetPose(); double trajectoryToleranceFuckery = .2;
    Pose2d initPose = drive.getPose2d();
    Pose2d placementPose = new Pose2d(
      initPose.getX()-.3-trajectoryToleranceFuckery, 
      initPose.getY(), 
      initPose.getRotation()
    );
    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
      initPose,  
      List.of(), 
      placementPose, 
      AutonControllers.revTrajectoryConfig
    );
    double yOffSet = .3*(DriverStation.getAlliance()==Alliance.Red?1:-1);
    Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
      placementPose, 
      List.of(
        new Translation2d(initPose.getX()+1,initPose.getY()+yOffSet)
      ),
      new Pose2d(initPose.getX()+4+trajectoryToleranceFuckery, initPose.getY()+yOffSet, initPose.getRotation()), 
      AutonControllers.trajectoryConfig
    );
    return new SequentialCommandGroup(
      boom.closeClaw(),
      boom.resetController(),
      boom.moveToBackTopPosition(),
      ramsetGenerator(drive, trajectory1),
      drive.stopCommand(),
      boom.openClaw(),
      new WaitCommand(1),
      ramsetGenerator(drive, trajectory2).alongWith(boom.moveToZeroPosition()),
      drive.stopCommand()
    );
  }
  
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");  
  }
  
}
