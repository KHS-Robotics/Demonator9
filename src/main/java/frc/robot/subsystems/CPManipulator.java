/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.vision.PixyCam;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CPManipulator extends SubsystemBase {
  private CANSparkMax motor;
  private double speed;
  private Solenoid solenoid;
  private final char[] colors = { 'R', 'G', 'B', 'Y', 'C' };

  public CPManipulator() {
    solenoid = new Solenoid(RobotMap.CP_SOLONOID);
    motor = new CANSparkMax(RobotMap.MANIPULATOR, MotorType.kBrushless);

    setPosition(false);
    var tab = Shuffleboard.getTab("Manipulator Speed");
    tab.addNumber("Speed", this::getSpeed);
    tab.addBoolean("Piston Up", solenoid::get);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spin(double speed) {
    motor.set(speed);
    this.speed = speed;
  }

  public double getSpeed() {
    return speed;
  }

  public void setPosition(boolean up) {
    solenoid.set(up);
  }

  public static char getColor() {
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
      case 'B':
        return 'B';
      case 'G':
        return 'G';
      case 'R':
        return 'R';
      case 'Y':
        return 'Y';
      default:
        return 'C';
      }
    } else {
      return ' ';
    }
  }
}
