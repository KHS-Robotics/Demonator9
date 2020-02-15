/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.vision.Limelight;
import frc.robot.RobotMap;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final double LIMELIGHT_ANGLE = 20.0; // 20 Degree Tilt
  
  private CANSparkMax leader, follower, hood;
  private CANPIDController shooterPid, hoodPid;
  private CANEncoder leaderEnc, followerEnc, hoodEnc;

  private double hoodPidSetpoint, shooterPidSetpoint;
  private boolean isClimbing;

  public Shooter() {
    leader = new CANSparkMax(RobotMap.SHOOTER1, MotorType.kBrushless);
    follower = new CANSparkMax(RobotMap.SHOOTER2, MotorType.kBrushless);
    hood = new CANSparkMax(RobotMap.HOOD, MotorType.kBrushless);

    follower.follow(leader);
    shooterPid = leader.getPIDController();
    hoodPid = hood.getPIDController();
    leaderEnc = leader.getEncoder();
    followerEnc = follower.getEncoder();
    hoodEnc = hood.getEncoder();

    leader.setIdleMode(IdleMode.kCoast);
    follower.setIdleMode(IdleMode.kCoast);

    hoodEnc.setPositionConversionFactor(360.0 / (10.0 * 5.0 * (60.0/24.0))); //Should be tested
    
    var tab = Shuffleboard.getTab("Shooter");
    tab.addNumber("Leader Speed", leaderEnc::getVelocity);
    tab.addNumber("Follower Speed", followerEnc::getVelocity);
    tab.addNumber("Hood Position", hoodEnc::getPosition);
    tab.addNumber("Hood Setpoint", () -> hoodPidSetpoint);
    tab.addNumber("Hood Error", () -> hoodPidSetpoint - hoodEnc.getPosition());
    tab.addNumber("Shooter Setpoint", () -> shooterPidSetpoint);
    tab.addNumber("Shooter Error", () -> shooterPidSetpoint - leaderEnc.getVelocity());
    tab.addBoolean("Is Climbing", () -> isClimbing);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot(double speed) {
    //shooterPid.setReference(speed, ControlType.kVelocity);
    leader.set(speed);
    shooterPidSetpoint = speed;
  }

  public void stop() {
    leader.set(0.0);
    hood.set(0.0);
    isClimbing = false;
  }

  public void stopShooter() {
    leader.set(0.0);
  }

  public void setHoodPid(double p, double i, double d) {
    hoodPid.setP(p);
    hoodPid.setI(i);
    hoodPid.setD(d);
  }

  public void setShooterPid(double p, double i, double d) {
    shooterPid.setP(p);
    shooterPid.setI(i);
    shooterPid.setD(d);
  }

  public void setHood(double angle) {
    //hoodPid.setReference(angle, ControlType.kPosition);
    hood.set(angle);
    hoodPidSetpoint = angle;
  }

  public void enableForClimb() {
    isClimbing = true;
    leader.set(0.15);
  }

  public void disableForClimb() {
    isClimbing = false;
    leader.set(0.0);
  }

  public double getVertAngle() {
    return LIMELIGHT_ANGLE + Limelight.getTy();
  }
}
