/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.rotate.RotateToAngle;
import frc.robot.commands.drive.rotate.RotateToTarget;
import frc.robot.commands.hood.AlignHoodToTarget;
import frc.robot.commands.indexer.SetIndexerAuto;
import frc.robot.commands.shooter.RampShooter;
import frc.robot.commands.shooter.ShootAuto;

/**
 * Add your docs here.
 */
public class AutoCommands {
  public static SwerveControllerCommand wallLineUp, frontTrench, pickTrench, returnTrench, moveOffInit;

  public static void autoInit() {
    wallLineUp = loadPathweaverTrajectory("output/WallLineUp.wpilib.json");
    frontTrench = loadPathweaverTrajectory("output/FrontOffset.wpilib.json");
    pickTrench = loadPathweaverTrajectory("output/PickTrench.wpilib.json");
    returnTrench = loadPathweaverTrajectory("output/ReturnFromLastBall.wpilib.json");
    moveOffInit = loadPathweaverTrajectory("output/MoveOffInit.wpilib.json");
  }
    
  public static SwerveControllerCommand loadPathweaverTrajectory(String json) {
      Trajectory trajectory;
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(json);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + json, ex.getStackTrace());
        return null;
      }
    
      return new SwerveControllerCommand(
        trajectory,
        RobotContainer.swerveDrive::getPose,
        RobotContainer.swerveDrive.kinematics,
        // Position controllers
        new PIDController(0.90, 0.001, 0.30), // x (forward/backwards)
        new PIDController(0.90, 0.001, 0.30), // y (side to side)
        new ProfiledPIDController(1.75, 0.001, 0.40, new TrapezoidProfile.Constraints(Math.PI, Math.PI)), // theta (rotation)
        RobotContainer.swerveDrive::setModuleStates,
        RobotContainer.swerveDrive
      );
  }

  public static Command sixBallAuto()  {
    return 
      wallLineUp
      .andThen(new RotateToTarget().alongWith(new AlignHoodToTarget()).alongWith(new RampShooter(-3000)))
      .andThen(new ShootAuto(-3000).alongWith(new SetIndexerAuto(0.45, -3000)).alongWith(new RotateToTarget()).withTimeout(5))
      .andThen(frontTrench)
      .andThen(pickTrench)
      .andThen((returnTrench).alongWith(new WaitCommand(0.5)))
      .andThen(new RotateToTarget().alongWith(new AlignHoodToTarget()))  
      .andThen(new ShootAuto(-3000).alongWith(new SetIndexerAuto(0.45, -3000)).alongWith(new RotateToTarget()).withTimeout(5));
  }

  public static Command shootOffInit()  {
    return 
      new WaitCommand(0.05)
      .andThen(new RotateToTarget().alongWith(new AlignHoodToTarget()).alongWith(new RampShooter(-3000)))
      .andThen(new ShootAuto(-3000).alongWith(new SetIndexerAuto(0.45, -3000)).alongWith(new RotateToTarget()).withTimeout(5))
      .andThen(moveOffInit);
  }
}