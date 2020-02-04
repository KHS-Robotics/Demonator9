/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CenterSwerveModules;
import frc.robot.commands.drive.DriveSwerveWithXbox;
import frc.robot.commands.drive.rotate.HoldAngleWhileDriving;
import frc.robot.commands.drive.rotate.RotateToTarget;
import frc.robot.commands.drive.rotate.RotateToTargetWhileDriving;
import frc.robot.subsystems.CPManipulator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
//import frc.robot.commands.PivotPIDTuner;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final static AHRS navx = new AHRS();

  public static final SwerveDrive swerveDrive = new SwerveDrive();
  public static final Intake intake = new Intake();
  public static final Climber climber = new Climber();
  public static final Shooter shooter = new Shooter();
  public static final CPManipulator CPManipulator = new CPManipulator();

  public static final XboxController xboxController = new XboxController(RobotMap.XBOX_PORT);
  public static final Joystick switchbox = new Joystick(RobotMap.SWITCHBOX_PORT);

  public static final DriveSwerveWithXbox driveSwerve = new DriveSwerveWithXbox();
  public static final RotateToTargetWhileDriving driveToTarget = new RotateToTargetWhileDriving();
  public static final RotateToTarget rotateToTargetInPlace = new RotateToTarget();
  public static final HoldAngleWhileDriving holdAngleWhileDriving = new HoldAngleWhileDriving();


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveDrive.setDefaultCommand(holdAngleWhileDriving);
    //swerveDrive.setDefaultCommand(new PivotPIDTuner());
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton calibrate = new JoystickButton(xboxController, XboxController.Button.kBack.value);
    calibrate.whenPressed(new CenterSwerveModules());

    CustomButton turnAndDrive = new CustomButton( () -> Math.abs(xboxController.getX(Hand.kLeft)) > 0.05 );
    turnAndDrive.whenHeld(driveSwerve);

    JoystickButton rotateToTarget = new JoystickButton(xboxController, XboxController.Button.kY.value);
    rotateToTarget.whenHeld(rotateToTargetInPlace);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}