/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Limelight;
import frc.robot.RobotContainer;

public class RotateToTarget extends RotateToAngle {
  double angle;
  double startTime;
  /**
   * Creates a new RotateToAngle.
   */
  public RotateToTarget() {
    super(0);
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.angle = RobotContainer.swerveDrive.getAngle().getDegrees() + Limelight.getTx();
    startTime = System.currentTimeMillis();
    RobotContainer.swerveDrive.stop();
    RobotContainer.swerveDrive.rotateToAngleInPlace(angle);
    //RobotContainer.swerveDrive.rotateToTargetInPlace();
  }
}
