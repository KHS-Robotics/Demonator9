/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IndexBall extends CommandBase {
  double position, startTime;
  boolean isDone = false, toggled = false;
  //firstBall = false

  /**
   * Creates a new IndexBall.
   */
  public IndexBall() {
    addRequirements(RobotContainer.indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
    toggled = false;
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!toggled) {
      RobotContainer.indexer.setMotor(0.75);
      if (!RobotContainer.indexer.getSwitch1()) {
        toggled = true;
      }
    } else {
      if (!RobotContainer.indexer.getSwitch2()) {
        RobotContainer.indexer.setMotor(0.75);
      } else {
        isDone = true;
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
