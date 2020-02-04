/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive.rotate;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.RobotContainer;

public class RotateToTarget extends CommandBase {
  double angle;
  /**
   * Creates a new RotateToAngle.
   */
  public RotateToTarget() {
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.swerveDrive.resetPid();

    
    this.angle = -RobotContainer.navx.getYaw() - Limelight.getTx();
    
    RobotContainer.swerveDrive.stop();
    RobotContainer.swerveDrive.rotateToAngleInPlace(angle);
    //RobotContainer.swerveDrive.rotateToTargetInPlace();
  }

  @Override
  public void execute() {
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.swerveDrive.atSetpoint();
  }
}
