/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.vision.Limelight.LightMode;
import frc.robot.vision.Limelight;


public class Robot extends TimedRobot {
  RobotContainer robotContainer;

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