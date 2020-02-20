/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.CenterSwerveModules;
import frc.robot.commands.drive.DriveSwerveWithXbox;
import frc.robot.commands.drive.rotate.HoldAngleWhileDriving;
import frc.robot.commands.drive.rotate.RotateToTargetWhileDriving;
import frc.robot.commands.indexer.ControlIndexer;
import frc.robot.commands.indexer.IndexBall;
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
  public static final Solenoid guide = new Solenoid(RobotMap.GUIDE);

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
    pixy.init();
    guide.set(false);
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

    JoystickButton rotateToTarget = new JoystickButton(xboxController, XboxController.Button.kY.value);
    //rotateToTarget.whenHeld(new RotateToTargetWhileDriving());

    // CustomButton turnAndDrive = new CustomButton( () -> Math.abs(xboxController.getX(Hand.kRight)) > 0.05 );
    // turnAndDrive.whileHeld(() -> {
    //   var xSpeed = -RobotContainer.xboxController.getY(GenericHID.Hand.kLeft) * SwerveDrive.kMaxSpeed;
    //   var ySpeed = -RobotContainer.xboxController.getX(GenericHID.Hand.kLeft) * SwerveDrive.kMaxSpeed;
    //   var rot = -RobotContainer.xboxController.getX(GenericHID.Hand.kRight) * SwerveDrive.kMaxAngularSpeed;

    //   if(Math.abs(xSpeed) > 0.05 || Math.abs(ySpeed) > 0.05 || Math.abs(rot) > 0.05) {
    //     RobotContainer.swerveDrive.drive(xSpeed, ySpeed, rot, !RobotContainer.xboxController.getBumper(GenericHID.Hand.kLeft));
    //   } else {
    //     RobotContainer.swerveDrive.stop();
    //   }
    // }, swerveDrive);

    CustomButton moveHood = new CustomButton(() -> switchbox.shooterOverride() && !switchbox.shoot());
    moveHood.whileHeld(() -> shooter.moveHood(switchbox.getHoodSpeed()), shooter);
    moveHood.whenReleased(() -> shooter.moveHood(0), shooter);

    CustomButton controlPanel = new CustomButton(switchbox::controlPanelOverride);
  controlPanel.whileHeld(() -> {
      CPManipulator.spin(switchbox.getControlPanel());
      CPManipulator.setPosition(true);
    }, CPManipulator);  
    controlPanel.whenReleased(() -> {
      CPManipulator.spin(0);
      CPManipulator.setPosition(false);
    }, CPManipulator);

    CustomButton setPto = new CustomButton(switchbox::engagePTO);
    setPto.whenPressed(() -> climber.setPTO(true), climber);
    setPto.whenReleased(() -> climber.setPTO(false), climber);

    CustomButton startClimb = new CustomButton(() -> (switchbox.climb() && switchbox.engagePTO()));
    startClimb.whenPressed(shooter::enableForClimb, shooter, climber);
    startClimb.whenReleased(shooter::disableForClimb, shooter, climber);

    CustomButton shoot = new CustomButton(() -> switchbox.shoot());
    shoot.whileHeld(() -> {
      shooter.setShooter(-4500);
      indexer.setMotor(.6);
    }, shooter, indexer);
    shoot.whenReleased(() -> {
      shooter.stop();
      indexer.stop();
    }, shooter, indexer);
    shoot.whenPressed(() -> shooter.setHood(shooter.getPosition()), shooter);

    CustomButton overrideHood = new CustomButton(() -> switchbox.shooterOverride());

    CustomButton intakeDown = new CustomButton(switchbox::intakeDown);
    intakeDown.whenPressed(intake::down, intake);
    intakeDown.whenReleased(intake::up, intake);

    CustomButton intaking = new CustomButton(switchbox::intake);
    intaking.whenPressed(() -> {
      intake.intake();
    }, intake);
    intaking.whenReleased(() -> {
      intake.stop();
    }, intake);

    CustomButton outtaking = new CustomButton(switchbox::outtake);
    outtaking.whenPressed(intake::reverse, intake);
    outtaking.whenReleased(intake::stop, intake);

    CustomButton guideButton = new CustomButton(switchbox::guide);
    guideButton.whenPressed(() -> guide.set(true));
    guideButton.whenReleased(() -> guide.set(false));

    CustomButton rotationControl = new CustomButton(() -> switchbox.rotationControl() && xboxController.getBButton());
    rotationControl.whenPressed(() -> {
      CPManipulator.spinNumTimes(CPManipulator.getPosition() + (8 * 3.5));
    }, CPManipulator);
    //rotationControl.whenReleased(() -> CPManipulator.setPosition(false));

    CustomButton controlPanelSwitch = new CustomButton(() -> switchbox.rotationControl() || switchbox.positionControl());
    controlPanelSwitch.whenPressed(() -> CPManipulator.setPosition(true), CPManipulator);
    controlPanelSwitch.whenReleased(() -> CPManipulator.setPosition(false), CPManipulator);    

    CustomButton positionControl = new CustomButton(() -> switchbox.positionControl() && xboxController.getBButton());
    //TODO: positionControl.whenHeld();

    CustomButton moveIndexer = new CustomButton(() -> (indexer.getSwitch1() && Math.abs(switchbox.getIndexSpeed()) < 0.05));
    moveIndexer.whenPressed(new IndexBall().withTimeout(2));

    CustomButton decreaseBall = new CustomButton(() -> (indexer.getSwitch1() && (switchbox.getIndexSpeed() < 0.05)));
    decreaseBall.whenPressed(indexer::decrementBall);

    CustomButton zeroBalls = new CustomButton(() -> (!switchbox.engagePTO() && switchbox.climb()));
    zeroBalls.whenPressed(indexer::zeroBalls);
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
