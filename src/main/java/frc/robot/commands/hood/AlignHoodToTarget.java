/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.vision.Limelight;
import frc.robot.vision.Limelight.LightMode;
import frc.robot.vision.table.InterpolatingDouble;
import frc.robot.vision.table.InterpolatingTreeMap;

public class AlignHoodToTarget extends CommandBase {
  static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> hoodAngleTable = new InterpolatingTreeMap<>();
  static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> lowHoodAngleTable = new InterpolatingTreeMap<>();
  static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> highHoodAngleTable = new InterpolatingTreeMap<>();

  static {
    hoodAngleTable.put(new InterpolatingDouble(0.07), new InterpolatingDouble(20.3));
    hoodAngleTable.put(new InterpolatingDouble(0.5), new InterpolatingDouble(21.33));
    hoodAngleTable.put(new InterpolatingDouble(2.0), new InterpolatingDouble(20.5));
    hoodAngleTable.put(new InterpolatingDouble(3.5), new InterpolatingDouble(20.5));
    hoodAngleTable.put(new InterpolatingDouble(4.6), new InterpolatingDouble(20.8));
    hoodAngleTable.put(new InterpolatingDouble(8.3), new InterpolatingDouble(20.2));
    hoodAngleTable.put(new InterpolatingDouble(9.2), new InterpolatingDouble(20.7));
    //hoodAngleTable.put(new InterpolatingDouble(10.9), new InterpolatingDouble(19.1));
    hoodAngleTable.put(new InterpolatingDouble(11.5), new InterpolatingDouble(20.6));

    // Low
    lowHoodAngleTable.put(new InterpolatingDouble(12.6), new InterpolatingDouble(20.25));
    lowHoodAngleTable.put(new InterpolatingDouble(13.7), new InterpolatingDouble(19.8));
    lowHoodAngleTable.put(new InterpolatingDouble(17.2), new InterpolatingDouble(19.66));
    lowHoodAngleTable.put(new InterpolatingDouble(17.6), new InterpolatingDouble(19.25));


    // High
    highHoodAngleTable.put(new InterpolatingDouble(-3.3), new InterpolatingDouble(21.9));
  }

  double ty;

  /**
   * Creates a new AlignHoodToTarget.
   */
  public AlignHoodToTarget() {
    addRequirements(RobotContainer.hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Limelight.setLedMode(LightMode.eOn);
    ty = Limelight.getTy();

    if (ty < -0.5) {
      InterpolatingDouble result = highHoodAngleTable.getInterpolated(new InterpolatingDouble(ty));
      if (result != null) {
        RobotContainer.hood.setHood(result.value);
      } else {
        RobotContainer.hood.setHood(RobotContainer.hood.getPosition());
      }
    } else if (ty > 12.2) {
      InterpolatingDouble result = lowHoodAngleTable.getInterpolated(new InterpolatingDouble(ty));
      if (result != null) {
        RobotContainer.hood.setHood(result.value);
      } else {
        RobotContainer.hood.setHood(RobotContainer.hood.getPosition());
      }
    } else {
      InterpolatingDouble result = hoodAngleTable.getInterpolated(new InterpolatingDouble(ty));
      if (result != null) {
        RobotContainer.hood.setHood(result.value);
      } else {
        RobotContainer.hood.setHood(RobotContainer.hood.getPosition());
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.hood.atSetpoint();
  }
}
