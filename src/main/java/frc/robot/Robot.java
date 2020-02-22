/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.vision.Limelight.LightMode;
import frc.robot.commands.CenterSwerveModules;
import frc.robot.vision.Limelight;


public class Robot extends TimedRobot {
  RobotContainer robotContainer;

  Command autonCommand;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    Limelight.setLedMode(LightMode.eOff);
  }

  @Override
  public void disabledPeriodic() {
    Limelight.setLedMode(LightMode.eOff);
  }

  @Override
  public void disabledInit() {
    RobotContainer.hood.hoodMode(false);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    RobotContainer.hood.hoodMode(true);

    RobotContainer.swerveDrive.resetNavx();

    if(autonCommand != null && autonCommand.isScheduled()) {
      autonCommand.cancel();
    }

    Command desiredAuton = robotContainer.getAutonomousCommand();

    if(!CenterSwerveModules.hasCalibrated()) {
      autonCommand = new CenterSwerveModules().andThen(desiredAuton);
    } else {
      autonCommand = desiredAuton;
    }

    if(autonCommand != null) {
      //autonCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    RobotContainer.swerveDrive.updateOdometry();
  }

  @Override
  public void teleopInit() {
    RobotContainer.hood.hoodMode(true);
  }

  @Override
  public void teleopPeriodic() {
  }
 
}