/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.CenterSwerveModules;
import frc.robot.commands.climb.SetPTO;
import frc.robot.commands.climb.StartClimb;
import frc.robot.commands.controlpanel.MoveControlPanel;
import frc.robot.commands.drive.DriveSwerveWithXbox;
import frc.robot.commands.drive.rotate.HoldAngleWhileDriving;
import frc.robot.commands.drive.rotate.RotateToTargetWhileDriving;
import frc.robot.commands.indexer.ControlIndexer;
import frc.robot.commands.intake.MoveDown;
import frc.robot.commands.intake.MoveUp;
import frc.robot.commands.intake.StartIntake;
import frc.robot.commands.intake.StartReverse;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooter.MoveHood;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.CPManipulator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

//import frc.robot.commands.PivotPIDTuner;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.links.SPILink;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static final Pixy2 pixy = Pixy2.createInstance(new SPILink());

  public static final AHRS navx = new AHRS();

  public static final SwerveDrive swerveDrive = new SwerveDrive();
  public static final Intake intake = new Intake();
  public static final Indexer indexer = new Indexer();
  public static final Climber climber = new Climber();
  public static final Shooter shooter = new Shooter();
  public static final CPManipulator CPManipulator = new CPManipulator();

  public static final XboxController xboxController = new XboxController(RobotMap.XBOX_PORT);
  public static final SwitchBox switchbox = new SwitchBox(RobotMap.SWITCHBOX_PORT);
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //swerveDrive.setDefaultCommand(new HoldAngleWhileDriving());
    swerveDrive.setDefaultCommand(new DriveSwerveWithXbox());
    indexer.setDefaultCommand(new ControlIndexer());
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

    // CustomButton turnAndDrive = new CustomButton( () -> Math.abs(xboxController.getX(Hand.kLeft)) > 0.05 );
    // turnAndDrive.whenHeld(new DriveSwerveWithXbox());

    JoystickButton rotateToTarget = new JoystickButton(xboxController, XboxController.Button.kY.value);
    rotateToTarget.whenHeld(new RotateToTargetWhileDriving());

    CustomButton moveHood = new CustomButton(switchbox::shooterOverride);
    moveHood.whenHeld(new MoveHood());

    CustomButton controlPanel = new CustomButton(switchbox::controlPanelOverride);
    controlPanel.whenHeld(new MoveControlPanel());

    CustomButton setPto = new CustomButton(switchbox::engagePTO);
    setPto.whenPressed(new SetPTO(true));
    setPto.whenReleased(new SetPTO(false));

    CustomButton startClimb = new CustomButton(switchbox::climb);
    startClimb.whenHeld(new StartClimb());

    CustomButton shoot = new CustomButton(switchbox::shoot);
    shoot.whenHeld(new Shoot());

    CustomButton intakeDown = new CustomButton(switchbox::intakeDown);
    intakeDown.whenPressed(new MoveDown());
    intakeDown.whenReleased(new MoveUp());

    CustomButton intaking = new CustomButton(switchbox::intake);
    intaking.whenPressed(new StartIntake());
    intaking.whenReleased(new StopIntake());

    CustomButton outtaking = new CustomButton(switchbox::outtake);
    outtaking.whenPressed(new StartReverse());
    outtaking.whenReleased(new StopIntake());

    CustomButton rotationControl = new CustomButton(() -> switchbox.rotationControl() && xboxController.getBButton());
    //TODO: rotationControl.whenHeld();

    CustomButton positionControl = new CustomButton(() -> switchbox.positionControl() && xboxController.getBButton());
    //TODO: positionControl.whenHeld();
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