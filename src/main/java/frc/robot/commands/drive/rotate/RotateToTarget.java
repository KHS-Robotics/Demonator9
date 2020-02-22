/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive.rotate;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.vision.Limelight;
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

    
    if(Limelight.isTarget()) {
      angle = -RobotContainer.navx.getYaw() - Limelight.getTx();
    } else {
      angle = 0;
    }
    
    RobotContainer.swerveDrive.stop();
  }

  @Override
  public void execute() {
    RobotContainer.swerveDrive.rotateToAngleInPlace(angle);
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.swerveDrive.atSetpoint();
  }
}
