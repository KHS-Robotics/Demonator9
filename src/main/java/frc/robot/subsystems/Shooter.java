/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  
  private CANSparkMax leader, follower, hood;
  private CANPIDController shooterPid, hoodPid;
  private CANEncoder leaderEnc, followerEnc, hoodEnc;

  private double hoodPidSetpoint;
  private boolean isClimbing;

  public Shooter() {
    leader = new CANSparkMax(RobotMap.SHOOTER1, MotorType.kBrushless);
    follower = new CANSparkMax(RobotMap.SHOOTER2, MotorType.kBrushless);
    hood = new CANSparkMax(RobotMap.HOOD, MotorType.kBrushless);

    follower.follow(leader); // TODO: might need inverted
    shooterPid = leader.getPIDController();
    hoodPid = hood.getPIDController();
    leaderEnc = leader.getEncoder();
    followerEnc = follower.getEncoder();
    hoodEnc = hood.getEncoder();

    var tab = Shuffleboard.getTab("Shooter");
    tab.addNumber("Leader Speed", leaderEnc::getVelocity);
    tab.addNumber("Follower Speed", followerEnc::getVelocity);
    tab.addNumber("Hood Setpoint", () -> hoodPidSetpoint);
    tab.addNumber("Hood Error", () -> hoodPidSetpoint - hoodEnc.getPosition());
    tab.addBoolean("Is Climbing", () -> isClimbing);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot() {
    leader.set(0.25);
  }

  public void stop() {
    leader.set(0.0);
    isClimbing = false;
  }

  public void setHood(double angle) {
    hoodPid.setReference(angle, ControlType.kPosition);
    hoodPidSetpoint = angle;
  }

  public void enableForClimb() {
    isClimbing = true;
    leader.set(0.5);
  }

  public void disableForClimb() {
    isClimbing = false;
    leader.set(0.0);
  }
}