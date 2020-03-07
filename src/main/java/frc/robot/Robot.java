/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.vision.Limelight.LightMode;
import frc.robot.commands.CenterSwerveModules;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.vision.Limelight;


public class Robot extends TimedRobot {
  RobotContainer robotContainer;

  Command autonCommand;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    RobotContainer.hood.hoodMode(false);
    RobotContainer.CPManipulator.brakeMode(false);
    SwerveDrive.kMaxSpeed = 3.5;
    SwerveDrive.kMaxAngularSpeed = Math.PI;
  }

  @Override
  public void disabledPeriodic() {
    Limelight.setLedMode(LightMode.eOff);
    
    if(RobotContainer.xboxController.getBButton() && RobotContainer.xboxController.getAButton() && RobotContainer.xboxController.getBumper(Hand.kLeft)) {
      RobotContainer.hood.resetEnc();
      RobotContainer.hood.stop();
    }
  }

  @Override
  public void autonomousInit() {
    int id = 1;

    Limelight.setLedMode(LightMode.eOn);
    RobotContainer.hood.hoodMode(true);
    RobotContainer.CPManipulator.brakeMode(true);

    RobotContainer.swerveDrive.resetNavx(robotContainer.getStartingPose(id));

    RobotContainer.shooter.setShooter(-3000);
    
    Command putIntakeDown = new InstantCommand(() -> RobotContainer.intake.down())
    .andThen(new WaitCommand(.5)
    .andThen(() -> RobotContainer.intake.setOff()));

    RobotContainer.indexer.setNumBalls(3);

    Command desiredAuton = 
      robotContainer.getAutonomousCommand(id)
      .alongWith(putIntakeDown)
      .andThen(() -> {
        RobotContainer.swerveDrive.stop();
        RobotContainer.shooter.stop();
        RobotContainer.hood.stop();
        RobotContainer.indexer.stop();
        RobotContainer.intake.stop();
    }, RobotContainer.swerveDrive, RobotContainer.shooter, RobotContainer.hood, RobotContainer.indexer, RobotContainer.intake);
    
    if(!CenterSwerveModules.hasCalibrated()) {
      autonCommand = new CenterSwerveModules().andThen(desiredAuton);
    } else {
      autonCommand = desiredAuton;
    }

    if(autonCommand != null) {
      autonCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    RobotContainer.swerveDrive.updateOdometry();
  }

  @Override
  public void teleopInit() {
    Limelight.setLedMode(LightMode.eOff);
    RobotContainer.hood.hoodMode(true);
    RobotContainer.CPManipulator.brakeMode(true);
    RobotContainer.shooter.stop();

    if(autonCommand != null) {
      autonCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    
  }
}