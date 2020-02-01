/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RotateToAngle extends CommandBase {
  double angle;
  double startTime;
  /**
   * Creates a new RotateToAngle.
   */
  public RotateToAngle(double angle) {
    this.angle = angle;
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    //RobotContainer.swerveDrive.rotateToTargetInPlace();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.swerveDrive.rotateToAngleInPlace(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.swerveDrive.atSetpoint() || this.isTimedOut();
  }

  private boolean isTimedOut() {
    return (System.currentTimeMillis() - startTime) > 1500;
  }
}
