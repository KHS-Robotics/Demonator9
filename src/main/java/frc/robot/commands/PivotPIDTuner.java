/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;

public class PivotPIDTuner extends CommandBase {
  double p, i, d;
  Rotation2d setPoint;
  SwerveModule sModule;

  public PivotPIDTuner(SwerveDrive swerve) {
    this.addRequirements(swerve);
    sModule = swerve.m_frontRight;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    SmartDashboard.putNumber("P-Value", 0.0);
    SmartDashboard.putNumber("I-Value", 0.0);
    SmartDashboard.putNumber("D-Value", 0.0);

    SmartDashboard.putNumber("Setpoint (Angle)", 0.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    p = SmartDashboard.getNumber("P-Value", 0.0);
    i = SmartDashboard.getNumber("I-Value", 0.0);
    d = SmartDashboard.getNumber("D-Value", 0.0);

    setPoint = Rotation2d.fromDegrees(SmartDashboard.getNumber("Setpoint (Angle)", 0.0));

    sModule.setPid(p, i, d);
    sModule.setDesiredState(new SwerveModuleState(0.0, setPoint));

    SmartDashboard.putNumber("Angle", sModule.getAngle());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stop();
  }
}