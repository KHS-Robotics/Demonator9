/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.vision.Limelight;
import frc.robot.Constants;
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

    setHoodPid(Constants.HOOD_P, Constants.HOOD_I, Constants.HOOD_D);
    setShooterPidF(Constants.SHOOTER_P, Constants.SHOOTER_I, Constants.SHOOTER_D, Constants.SHOOTER_FF);

    shooterPid.setIZone(500);
    shooterPid.setOutputRange(-1, 0);

    leader.setIdleMode(IdleMode.kCoast);
    follower.setIdleMode(IdleMode.kCoast);

    hood.setIdleMode(IdleMode.kBrake);
    // hoodEnc.setPositionConversionFactor(360.0 / (10.0 * 5.0 * (60.0/24.0))); //Should be tested
    
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

  public void setShooter(double speed) {
    shooterPid.setReference(speed, ControlType.kVelocity);
    shooterPidSetpoint = speed;
  }

  public void setShooterWithoutPID(double speed) {
    leader.set(speed);
  }

  public void moveHood(double speed) {
    hood.set(speed);
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

  public void setShooterPidF(double p, double i, double d, double ff) {
    shooterPid.setP(p);
    shooterPid.setI(i);
    shooterPid.setD(d);
    shooterPid.setFF(ff);
  }

  public void setHood(double angle) {
    hoodPid.setReference(angle, ControlType.kPosition);
    hoodPidSetpoint = angle;
  }

  public double getPosition() {
    return hoodEnc.getPosition();
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

  public void hoodMode(boolean brake) {
    if(brake) {
      hood.setIdleMode(IdleMode.kBrake);
    } else {
      hood.setIdleMode(IdleMode.kCoast);
    }
  }

  public double getCurrent() {
    return leader.getOutputCurrent();
  }
}
