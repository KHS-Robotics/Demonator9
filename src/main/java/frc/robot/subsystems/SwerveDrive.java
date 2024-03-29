/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.commands.CenterSwerveModules;

/**
 * Represents a swerve drive style drivetrain.
 */
public class SwerveDrive extends SubsystemBase {
  public static double kMaxSpeed = 3.5; // 3.5 meters per second
  public static double kMaxAngularSpeed = Math.PI; // pi radians per second
  public static double offset;
  private PIDController targetPid;
  private final Translation2d frontLeftLocation = new Translation2d(0.29845, 0.29845);
  private final Translation2d frontRightLocation = new Translation2d(0.29845, -0.29845);
  private final Translation2d rearLeftLocation = new Translation2d(-0.29845, 0.29845);
  private final Translation2d rearRightLocation = new Translation2d(-0.29845, -0.29845);

  public static final SwerveModule frontLeft = new SwerveModule(
    "FL", 
    RobotMap.FRONT_LEFT_DRIVE,
    RobotMap.FRONT_LEFT_PIVOT, 
    Constants.FRONT_LEFT_P, 
    Constants.FRONT_LEFT_I, 
    Constants.FRONT_LEFT_D, 
    Constants.FRONT_LEFT_DRIVE_P, 
    Constants.FRONT_LEFT_DRIVE_I, 
    Constants.FRONT_LEFT_DRIVE_D,
    Constants.FRONT_LEFT_DRIVE_FF,
    RobotMap.FRONT_LEFT_DIGITAL_INPUT,
    true
  );
  public static final SwerveModule frontRight = new SwerveModule(
    "FR", 
    RobotMap.FRONT_RIGHT_DRIVE,
    RobotMap.FRONT_RIGHT_PIVOT, 
    Constants.FRONT_RIGHT_P, 
    Constants.FRONT_RIGHT_I, 
    Constants.FRONT_RIGHT_D, 
    Constants.FRONT_RIGHT_DRIVE_P,
    Constants.FRONT_RIGHT_DRIVE_I, 
    Constants.FRONT_RIGHT_DRIVE_D,
    Constants.FRONT_RIGHT_DRIVE_FF,
    RobotMap.FRONT_RIGHT_DIGITAL_INPUT
  );
  public static final SwerveModule rearLeft = new SwerveModule(
    "RL", 
    RobotMap.REAR_LEFT_DRIVE,
    RobotMap.REAR_LEFT_PIVOT, 
    Constants.REAR_LEFT_P, 
    Constants.REAR_LEFT_I, 
    Constants.REAR_LEFT_D, 
    Constants.REAR_LEFT_DRIVE_P,
    Constants.REAR_LEFT_DRIVE_I, 
    Constants.REAR_LEFT_DRIVE_D,
    Constants.REAR_LEFT_DRIVE_FF,
    RobotMap.REAR_LEFT_DIGITAL_INPUT,
    true
  );
  public static final SwerveModule rearRight = new SwerveModule(
    "RR",
    RobotMap.REAR_RIGHT_DRIVE,
    RobotMap.REAR_RIGHT_PIVOT, 
    Constants.REAR_RIGHT_P, 
    Constants.REAR_RIGHT_I, 
    Constants.REAR_RIGHT_D, 
    Constants.REAR_RIGHT_DRIVE_P, 
    Constants.REAR_RIGHT_DRIVE_I, 
    Constants.REAR_RIGHT_DRIVE_D,
    Constants.REAR_RIGHT_DRIVE_FF, 
    RobotMap.REAR_RIGHT_DIGITAL_INPUT 
  );

  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation,
      frontRightLocation, rearLeftLocation, rearRightLocation);

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, this.getAngle());

  /**
   * Constructs Swerve Drive
   */
  public SwerveDrive() {
    targetPid = new PIDController(Constants.TARGET_P, Constants.TARGET_I, Constants.TARGET_D);
    targetPid.enableContinuousInput(-180.0, 180.0);
    targetPid.setTolerance(1);

    var tab = Shuffleboard.getTab("Swervedrive");
    tab.addNumber("Target Angle", targetPid::getSetpoint);
    tab.addNumber("Target Error", targetPid::getPositionError);
    tab.addNumber("Current Angle", () -> getYaw());
    tab.addNumber("Angle Graph", () -> getYaw());
    tab.addNumber("Pose X", () -> this.getPose().getTranslation().getX());
    tab.addNumber("Pose Y", () -> this.getPose().getTranslation().getY());
    tab.addNumber("Pose Norm", () -> this.getPose().getTranslation().getNorm());
    tab.addNumber("Pose Rotation", () -> this.getPose().getRotation().getDegrees());
    tab.addBoolean("Calibrated", CenterSwerveModules::hasCalibrated);

    var matchTab = Shuffleboard.getTab("Match");
    matchTab.addBoolean("Calibrated", CenterSwerveModules::hasCalibrated); 
  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(-RobotContainer.navx.getAngle() + offset);
  }

  public double getYaw() {
    return normalizeAngle(-RobotContainer.navx.getYaw() + offset);
  }

  public void setOffset(double offset) {
    SwerveDrive.offset = offset;
  }

  public double sensControl(double var) {
    return Constants.SENS * Math.pow(var, 3) + (1 - Constants.SENS) * var;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward) in m/s.
   * @param ySpeed        Speed of the robot in the y direction (sideways) in m/s.
   * @param rot           Angular rate of the robot in rad/sec.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if(Math.abs(rot) < 0.005 && Math.abs(xSpeed) < 0.015 && Math.abs(ySpeed) < 0.015) {
      frontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(frontLeft.getAngle())));
      frontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(frontRight.getAngle())));
      rearLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(rearLeft.getAngle())));
      rearRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(rearRight.getAngle())));
    } else {
      var swerveModuleStates = kinematics
        .toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);

      frontLeft.setDesiredState(swerveModuleStates[0]);
      frontRight.setDesiredState(swerveModuleStates[1]);
      rearLeft.setDesiredState(swerveModuleStates[2]);
      rearRight.setDesiredState(swerveModuleStates[3]);
    }

    // System.out.println("FR " + swerveModuleStates[0].angle.getDegrees());
    // System.out.println("FL " + swerveModuleStates[1].angle.getDegrees());
    // System.out.println("RL" + swerveModuleStates[2].angle.getDegrees());
    // System.out.println("RR " + swerveModuleStates[3].angle.getDegrees());
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, kMaxSpeed);
    frontLeft.setDesiredState(desiredStates[0], true);
    frontRight.setDesiredState(desiredStates[1], true);
    rearLeft.setDesiredState(desiredStates[2], true);
    rearRight.setDesiredState(desiredStates[3], true);
  }


  @Override
  public void periodic() {
    // var pose = this.getPose();
  }

  public void setPID(double p, double i, double d) {
    targetPid.setPID(p, i, d);
  }

  public void rotateToAngleInPlace(double setAngle) {
    holdAngleWhileDriving(0, 0, setAngle, false);
  }

  public void holdAngleWhileDriving(double x, double y, double setAngle, boolean fod) {
    var rotateOutput = MathUtil.clamp(targetPid.calculate(getYaw(), normalizeAngle(setAngle)), -1, 1) * kMaxAngularSpeed;
    this.drive(x, y, rotateOutput, fod);
  }

  public boolean atSetpoint() {
    return targetPid.atSetpoint();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  /**
   * Updates the field relative position of the robot.
   */
  public void updateOdometry() {
    odometry.update(this.getAngle(), frontLeft.getState(), frontRight.getState(), rearLeft.getState(),
        rearRight.getState());
  }

  public void stop() {
    frontRight.stop();
    frontLeft.stop();
    rearRight.stop();
    rearLeft.stop();
  }

  public void resetNavx() {
    resetNavx(new Pose2d(0,0,Rotation2d.fromDegrees(0)));
  }

  public void resetNavx(Pose2d currentPose) {
    targetPid.reset();
    RobotContainer.navx.reset();
    odometry.resetPosition(currentPose, getAngle()); 
  }

  public void resetPid() {
    targetPid.reset();
  }

  public boolean resetEncoders() { 
    boolean fl = frontLeft.resetEncoder();
    boolean fr = frontRight.resetEncoder();
    boolean rl = rearLeft.resetEncoder();
    boolean rr = rearRight.resetEncoder();
    return fl && fr && rl && rr;
  }

  private static double normalizeAngle(double angle) {
    if(angle > 0) {
      angle %= 360;
      if(angle > 180) {
        angle -= 360;
      }
    }
    else if(angle < 0) {
      angle %= -360;
      if(angle < -180) {
        angle += 360;
      }
    }
    return angle;
  }
}
