/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Limelight.LightMode;

public class Robot extends TimedRobot {
  RobotContainer robotContainer;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    Limelight.setLedMode(LightMode.off)
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
    if (RobotContainer.xboxController.getStartButton()) {
      RobotContainer.swerveDrive.resetNavx();
    }
  }
 
}