/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

/**
 * Represents a swerve drive style drivetrain.
 */
public class SwerveDrive extends SubsystemBase {
  public static final double kMaxSpeed = 1.0; // 1 meters per second
  public static final double kMaxAngularSpeed = 1; // 1 radian per second

  private final Translation2d frontLeftLocation = new Translation2d(0.5969, 0.5969);
  private final Translation2d frontRightLocation = new Translation2d(0.5969, -0.5969);
  private final Translation2d rearLeftLocation = new Translation2d(-0.5969, 0.5969);
  private final Translation2d rearRightLocation = new Translation2d(-0.5969, -0.5969);

  public static final SwerveModule frontLeft = new SwerveModule(
    "FL", 
    RobotMap.FRONT_LEFT_DRIVE,
    RobotMap.FRONT_LEFT_PIVOT, 
    Constants.FRONT_LEFT_P, 
    Constants.FRONT_LEFT_I, 
    Constants.FRONT_LEFT_D, 
    RobotMap.FRONT_LEFT_DIGITAL_INPUT
  );
  public static final SwerveModule frontRight = new SwerveModule(
    "FR", 
    RobotMap.FRONT_RIGHT_DRIVE,
    RobotMap.FRONT_RIGHT_PIVOT, 
    Constants.FRONT_RIGHT_P, 
    Constants.FRONT_RIGHT_I, 
    Constants.FRONT_RIGHT_D, 
    RobotMap.FRONT_RIGHT_DIGITAL_INPUT
  );
  public static final SwerveModule rearLeft = new SwerveModule(
    "RL", 
    RobotMap.REAR_LEFT_DRIVE,
    RobotMap.REAR_LEFT_PIVOT, 
    Constants.REAR_LEFT_P, 
    Constants.REAR_LEFT_I, 
    Constants.REAR_LEFT_D, 
    RobotMap.REAR_LEFT_DIGITAL_INPUT
  );
  public static final SwerveModule rearRight = new SwerveModule(
    "RR",
    RobotMap.REAR_RIGHT_DRIVE,
    RobotMap.REAR_RIGHT_PIVOT, 
    Constants.REAR_RIGHT_P, 
    Constants.REAR_RIGHT_I, 
    Constants.REAR_RIGHT_D, 
    RobotMap.REAR_RIGHT_DIGITAL_INPUT 
  );

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation,
      frontRightLocation, rearLeftLocation, rearRightLocation);

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, this.getAngle());

  /**
   * Constructs Swerve Drive
   */
  public SwerveDrive() {
    RobotContainer.navx.reset();
  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(-RobotContainer.navx.getAngle());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = kinematics
        .toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
    // System.out.println("FR " + swerveModuleStates[0].angle.getDegrees());
    // System.out.println("FL " + swerveModuleStates[1].angle.getDegrees());
    // System.out.println("RL" + swerveModuleStates[2].angle.getDegrees());
    // System.out.println("RR " + swerveModuleStates[3].angle.getDegrees());
  }

  @Override

  public void periodic() {
    // var pose = this.getPose();
    // SmartDashboard.putNumber("Pose-X (meters)", pose.getTranslation().getX());
    // SmartDashboard.putNumber("Pose-Y (meters)", pose.getTranslation().getY());
    // SmartDashboard.putNumber("Pose-Norm (meters)", pose.getTranslation().getNorm());
    // SmartDashboard.putNumber("Pose-Rotation (Deg)", pose.getRotation().getDegrees());

    SmartDashboard.putNumber("Robot Angle (Deg)", getAngle().getDegrees());
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
    RobotContainer.navx.reset();
  }

  public boolean resetEncoders() { 
    boolean fl = frontLeft.resetEncoder();
    boolean fr = frontRight.resetEncoder();
    boolean rl = rearLeft.resetEncoder();
    boolean rr = rearRight.resetEncoder();
    return fl && fr && rl && rr;
  }
}