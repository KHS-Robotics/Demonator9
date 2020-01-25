/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveSwerveWithXbox;

public class Robot extends TimedRobot {

  @Override
  public void robotInit() {
    RobotContainer.swerveDrive.setDefaultCommand(new DriveSwerveWithXbox());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousPeriodic() {
    //driveWithJoystick(false);
    RobotContainer.swerveDrive.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    //driveWithJoystick(false);
  }
 
}