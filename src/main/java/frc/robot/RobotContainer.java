/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.CenterSwerveModules;
import frc.robot.commands.drive.DriveSwerveWithXbox;
import frc.robot.commands.drive.rotate.HoldAngleWhileDriving;
import frc.robot.commands.drive.rotate.RotateToTargetWhileDriving;
import frc.robot.commands.indexer.ControlIndexer;
import frc.robot.commands.indexer.IndexBall;
import frc.robot.commands.indexer.SetIndexer;
import frc.robot.commands.pid.TargetPIDTuner;
import frc.robot.commands.shooter.HoldHoodAngle;
import frc.robot.commands.shooter.RampShooter;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.CPManipulator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hood;
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
  public static final Hood hood = new Hood();
  public static final CPManipulator CPManipulator = new CPManipulator();
  public static final Solenoid guide = new Solenoid(RobotMap.GUIDE);
  public static final PowerDistributionPanel pdp = new PowerDistributionPanel();

  public static final XboxController xboxController = new XboxController(RobotMap.XBOX_PORT);
  public static final SwitchBox switchbox = new SwitchBox(RobotMap.SWITCHBOX_PORT);
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveDrive.setDefaultCommand(new HoldAngleWhileDriving());
    //swerveDrive.setDefaultCommand(new TargetPIDTuner());
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
    Button autoCalibrate = new Button(() -> (!CenterSwerveModules.hasCalibrated()));
    autoCalibrate.whenPressed(new CenterSwerveModules());

    //Button autoCal = new Button(() -> !CenterSwerveModules.hasCalibrated() && RobotState.isAutonomous() && RobotState.isEnabled());
    //autoCal.whenPressed(new CenterSwerveModules());

    JoystickButton calibrate = new JoystickButton(xboxController, XboxController.Button.kBack.value);
    calibrate.whenPressed(new CenterSwerveModules());

    JoystickButton rotateToTarget = new JoystickButton(xboxController, XboxController.Button.kY.value);
    rotateToTarget.whenHeld(new RotateToTargetWhileDriving());

    Button turnAndDrive = new Button( () -> Math.abs(xboxController.getX(Hand.kRight)) > 0.01 && CenterSwerveModules.hasCalibrated() && !switchbox.unusedSwitch());
    turnAndDrive.whileHeld(() -> { 
      var xSpeed = swerveDrive.sensControl(-RobotContainer.xboxController.getY(GenericHID.Hand.kLeft)) * SwerveDrive.kMaxSpeed;
      var ySpeed = swerveDrive.sensControl(-RobotContainer.xboxController.getX(GenericHID.Hand.kLeft)) * SwerveDrive.kMaxSpeed;
      var rot = swerveDrive.sensControl(-RobotContainer.xboxController.getX(GenericHID.Hand.kRight)) * SwerveDrive.kMaxAngularSpeed;

      if(Math.abs(xSpeed) > 0.01 || Math.abs(ySpeed) > 0.01 || Math.abs(rot) > 0.01) {
        RobotContainer.swerveDrive.drive(xSpeed, ySpeed, rot, !RobotContainer.xboxController.getBumper(GenericHID.Hand.kLeft));
      } else {
        RobotContainer.swerveDrive.stop();
      }
    }, swerveDrive);

    Button moveHood = new Button(() -> switchbox.shooterOverride() && !switchbox.shoot());
    moveHood.whileHeld(() -> hood.moveHood(switchbox.getHoodSpeed()), hood);
    moveHood.whenReleased(() -> hood.moveHood(0), hood);

    Button controlPanel = new Button(switchbox::controlPanelOverride);
    controlPanel.whileHeld(() -> {
      CPManipulator.spin(switchbox.getControlPanel());
      CPManipulator.setPosition(true);
    }, CPManipulator);  
    controlPanel.whenReleased(() -> {
      CPManipulator.spin(0);
      CPManipulator.setPosition(false);
    }, CPManipulator);

    Button setPto = new Button(switchbox::engagePTO);
    setPto.whenPressed(() -> climber.setPTO(true), climber);
    setPto.whenReleased(() -> climber.setPTO(false), climber);

    Button startClimb = new Button(() -> (switchbox.climb() && switchbox.engagePTO()));
    startClimb.whenPressed(shooter::enableForClimb, shooter, climber);
    startClimb.whenReleased(shooter::disableForClimb, shooter, climber);

    Button shoot = new Button(() -> switchbox.shoot());
    shoot.whileHeld(new RampShooter(-4500).andThen(new Shoot(-4500)).alongWith(new SetIndexer(0.6)).alongWith(new HoldHoodAngle()));
    shoot.whenReleased(() -> {
      shooter.stop();
      hood.stop();
      indexer.stop();
    }, shooter, hood, indexer);
    shoot.whenPressed(() -> hood.setHood(hood.getPosition()), hood);

    Button overrideHood = new Button(() -> switchbox.shooterOverride());

    Button intakeDown = new Button(switchbox::intakeDown);
    intakeDown.whenPressed(intake::down, intake);
    intakeDown.whenPressed(new WaitCommand(0.5).andThen(() -> intake.setOff()));
    intakeDown.whenReleased(intake::up, intake);
    intakeDown.whenReleased(new WaitCommand(0.5).andThen(() -> intake.setOff()));

    Button intaking = new Button(() -> (switchbox.intake() && indexer.getNumBalls() < 5));
    intaking.whenPressed(() -> {
      intake.intake();
    }, intake);
    intaking.whenReleased(() -> {
      intake.stop();
    }, intake);

    Button outtaking = new Button(switchbox::outtake);
    outtaking.whenPressed(intake::reverse, intake);
    outtaking.whenReleased(intake::stop, intake);

    Button guideButton = new Button(switchbox::guide);
    guideButton.whenPressed(() -> guide.set(true));
    guideButton.whenReleased(() -> guide.set(false));

    Button rotationControl = new Button(() -> switchbox.rotationControl() && xboxController.getBButton());
    rotationControl.whenPressed(() -> {
      CPManipulator.spinNumTimes(CPManipulator.getPosition() + (8 * 4.8));
    }, CPManipulator);
    //rotationControl.whenReleased(() -> CPManipulator.setPosition(false));

    Button controlPanelSwitch = new Button(() -> switchbox.rotationControl() || switchbox.positionControl());
    controlPanelSwitch.whenPressed(() -> CPManipulator.setPosition(true), CPManipulator);
    controlPanelSwitch.whenReleased(() -> CPManipulator.setPosition(false), CPManipulator);    

    Button lampOn = new Button(() -> switchbox.positionControl());
    lampOn.whenPressed(() -> pixy.setLamp((byte) 1, (byte) 1));
    lampOn.whenReleased(() -> pixy.setLamp((byte) 0, (byte) 0));
    lampOn.whileHeld(() -> CPManipulator.update());

    Button positionControl = new Button(() -> switchbox.positionControl() && xboxController.getBButton());

    Button moveIndexer = new Button(() -> (indexer.getSwitch1() && Math.abs(switchbox.getIndexSpeed()) < 0.05));
    //moveIndexer.whenPressed(new IndexBall().withTimeout(1));

    Button decreaseBall = new Button(() -> (indexer.getSwitch1() && (switchbox.getIndexSpeed() < -0.05)));
    //decreaseBall.whenPressed(indexer::decrementBall);

    Button zeroBalls = new Button(() -> (!switchbox.engagePTO() && switchbox.climb()));
    //zeroBalls.whenPressed(indexer::zeroBalls);

    Button unusedButton = new Button(() -> switchbox.unusedSwitch());
    unusedButton.whileHeld(() -> swerveDrive.stop(), swerveDrive);

    Button resetNavx = new Button(() -> (RobotContainer.xboxController.getStartButton()));
    resetNavx.whenPressed(() -> RobotContainer.swerveDrive.resetNavx(), swerveDrive);
    
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(1, 1)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(swerveDrive.kinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config
    );

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        swerveDrive::getPose, //Functional interface to feed supplier
        swerveDrive.kinematics,

        //Position controllers
        new PIDController(.25, 0, 0),
        new PIDController(.25, 0, 0),
        new ProfiledPIDController(.25, 0, 0, new TrapezoidProfile.Constraints(1,1)),

        swerveDrive::setModuleStates,

        swerveDrive

    );

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> swerveDrive.stop());
  }
}