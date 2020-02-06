/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.vision.ColorBlock;
import frc.robot.vision.ColorWheel;
import frc.robot.vision.PixyCam;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

import java.util.ArrayList;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CPManipulator extends SubsystemBase {

  /**
   * Spin specified number of times
   */

  private CANSparkMax motor;
  private CANEncoder motorEnc;
  private CANPIDController motorPid;
  private double speed;
  private Solenoid solenoid;
  private int currentColorSignature;
  private double curPos;
  private double curRPM;

  public CPManipulator() {
    solenoid = new Solenoid(RobotMap.CP_SOLONOID);
    motor = new CANSparkMax(RobotMap.MANIPULATOR, MotorType.kBrushless);
    motorEnc = motor.getEncoder();
    motorPid = motor.getPIDController();

    setPosition(false);
    var tab = Shuffleboard.getTab("Manipulator Speed");
    tab.addNumber("Speed", this::getSpeed);
    tab.addBoolean("Piston Up", solenoid::get);

    motorPid.setP(0.0);
    motorPid.setI(0.0);
    motorPid.setD(0.0);
    motorPid.setIZone(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentColorSignature = getCurColor();
    curPos = motorEnc.getPosition();
    curRPM = motorEnc.getVelocity();
  }

  public int distToColor(char curColor, char toColor) {
    int dist, curIndex, toIndex;

    curIndex = ColorWheel.toColor(curColor).signature;
    toIndex = ColorWheel.toColor(toColor).signature;

    dist = toIndex - curIndex;

    return dist;
  }

  public double distToSpins(int dist) {
    double spins = dist / 8.0;

    return spins;
  }

  public double spinsToRadians(double spins) {
    double degrees = spins * 360;
    double radians = (degrees * Math.PI) / 180;
    return radians;
  }

  public double spinsToColor(char curColor, char toColor) {
    return distToSpins(distToColor(curColor, toColor));
  }

  public void spin(double speed) {
    motorPid.setReference(speed, ControlType.kSmartMotion);
    this.speed = speed;
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

  public int getCurColor() {
    int curColorSig = -1;
    ArrayList<ColorBlock> newBlocks = new ArrayList<ColorBlock>(0);

    ArrayList<Block> blocks = PixyCam.getBlocks();

    for (int i = 0; i < blocks.size(); i++) {
      ColorBlock newBlock = new ColorBlock(blocks.get(i));
      newBlocks.set(i, newBlock);
    }

    newBlocks = PixyCam.sortByWeight(newBlocks);

    if (newBlocks.size() > 0) {
      curColorSig = newBlocks.get(0).getSig();
    }

    return curColorSig;
  }

  public double getSpeed() {
    return speed;
  }

  public void setPosition(boolean up) {
    solenoid.set(up);
  }
}