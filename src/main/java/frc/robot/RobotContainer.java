/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Servo;

import frc.robot.commands.CenterSwerveModules;
import frc.robot.commands.drive.DriveSwerveWithXbox;
import frc.robot.commands.drive.rotate.HoldAngleWhileDriving;
import frc.robot.commands.drive.rotate.RotateToTargetWhileDriving;
import frc.robot.commands.indexer.ControlIndexer;
import frc.robot.commands.indexer.IndexBall;
import frc.robot.commands.indexer.SetIndexer;
import frc.robot.commands.pid.TargetPIDTuner;
import frc.robot.commands.hood.HoldHoodAngle;
import frc.robot.commands.hood.MoveHood;
import frc.robot.commands.hood.MoveHoodDown;
import frc.robot.commands.hood.SetHoodAngle;
import frc.robot.commands.shooter.RampShooter;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.CPManipulator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.links.SPILink;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
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
  public static final Servo servo = new Servo(RobotMap.CAMERA_SERVO);
  public static final PowerDistributionPanel pdp = new PowerDistributionPanel();

  public static final XboxController xboxController = new XboxController(RobotMap.XBOX_PORT);
  public static final SwitchBox switchbox = new SwitchBox(RobotMap.SWITCHBOX_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveDrive.setDefaultCommand(new DriveSwerveWithXbox());
    // swerveDrive.setDefaultCommand(new TargetPIDTuner());
    indexer.setDefaultCommand(new ControlIndexer());
    // swerveDrive.setDefaultCommand(new PivotPIDTuner());
    pixy.init();
    guide.set(false);
    servo.set(.225);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Button autoCalibrateTeleop = new Button(
        () -> (!CenterSwerveModules.hasCalibrated() && RobotState.isOperatorControl() && RobotState.isEnabled()));
    autoCalibrateTeleop.whenPressed(new CenterSwerveModules());

    JoystickButton forceCalibrate = new JoystickButton(xboxController, XboxController.Button.kBack.value);
    forceCalibrate.whenPressed(new CenterSwerveModules());

    // Button unusedButton = new Button(() -> switchbox.unusedSwitch());
    // unusedButton.whileHeld(() -> swerveDrive.stop(), swerveDrive);

    JoystickButton rotateToTarget = new JoystickButton(xboxController, XboxController.Button.kY.value);
    rotateToTarget.whenHeld(new RotateToTargetWhileDriving());

    /*
     * Button turnAndDrive = new Button(() ->
     * Math.abs(xboxController.getX(Hand.kRight)) > 0.01 &&
     * CenterSwerveModules.hasCalibrated() && !switchbox.unusedSwitch());
     * turnAndDrive.whileHeld(() -> { var xSpeed =
     * swerveDrive.sensControl(-RobotContainer.xboxController.getY(GenericHID.Hand.
     * kLeft)) SwerveDrive.kMaxSpeed; var ySpeed =
     * swerveDrive.sensControl(-RobotContainer.xboxController.getX(GenericHID.Hand.
     * kLeft)) SwerveDrive.kMaxSpeed; var rot =
     * swerveDrive.sensControl(-RobotContainer.xboxController.getX(GenericHID.Hand.
     * kRight)) SwerveDrive.kMaxAngularSpeed;
     * 
     * RobotContainer.swerveDrive.drive(xSpeed, ySpeed, rot,
     * !RobotContainer.xboxController.getBumper(GenericHID.Hand.kLeft)); },
     * swerveDrive);
     */

    Button holdAngle = new Button(() -> xboxController.getAButton());
    holdAngle.whenHeld(new HoldAngleWhileDriving());

    Button moveHood = new Button(() -> switchbox.shooterOverride() && !switchbox.shoot());
    moveHood.whileHeld(() -> hood.moveHood(switchbox.getHoodSpeed()), hood);
    moveHood.whenReleased(() -> hood.moveHood(0), hood);

    Button controlPanel = new Button(switchbox::controlPanelOverride);
    controlPanel.whileHeld(() -> {
      CPManipulator.spin(switchbox.getControlPanel());
      // CPManipulator.setPosition(true);
    }, CPManipulator);
    controlPanel.whenReleased(() -> {
      CPManipulator.spin(0);
      // CPManipulator.setPosition(false);
    }, CPManipulator);

    Button setPto = new Button(switchbox::engagePTO);
    setPto.whenPressed(() -> {
      climber.setPTO(true);
      servo.set(0.8);
    }, climber);
    setPto.whenReleased(() -> {
      climber.setPTO(false);
      servo.set(0.225);
    }, climber);

    Button telescope = new Button(() -> switchbox.engagePTO());
    telescope.whileHeld(() -> {
      var speed = switchbox.getTelescopeSpeed();
      climber.setTelescope(Math.abs(speed) > 0.03 ? speed : 0.0);
    }, climber);
    telescope.whenReleased(() -> climber.setTelescope(0), climber);

    Button startClimb = new Button(() -> (switchbox.engagePTO() && switchbox.climb()));
    startClimb.whenPressed(shooter::enableForClimb, shooter, climber);
    startClimb.whenReleased(shooter::disableForClimb, shooter, climber);

    Button rampShooterWithoutGuide = new Button(() -> switchbox.rampShooter() && !switchbox.guide());
    rampShooterWithoutGuide.whenPressed(new RampShooter(-3000));

    Button rampShooterWithGuide = new Button(() -> switchbox.rampShooter() && switchbox.guide());
    rampShooterWithGuide.whenPressed(new RampShooter(-4500));

    Button releaseShooter = new Button(() -> !switchbox.rampShooter());
    releaseShooter.whenPressed(() -> shooter.stop(), shooter);

    Button manualShoot = new Button(() -> switchbox.shoot() && !switchbox.guide());
    manualShoot.whenPressed(new RampShooter(-3000).alongWith(new HoldHoodAngle())
        .andThen(new Shoot(-3000).alongWith(new SetIndexer(0.45, -3000))));
    manualShoot.whenReleased(() -> {
      shooter.stop();
      hood.stop();
      indexer.stop();
    }, shooter, hood, indexer);

    /*
     * Button shootWithVisionClose = new Button(() -> !switchbox.guide() &&
     * !switchbox.shooterOverride() && switchbox.shoot());
     * shootWithVisionClose.whenPressed( new AngleHoodToTarget() .alongWith(new
     * RampShooter(-3000)) .andThen(new Shoot(-3000).alongWith(new
     * SetIndexer(0.45))) ); shootWithVisionClose.whenReleased(() -> {
     * shooter.stop(); hood.stop(); indexer.stop(); }, shooter, hood, indexer);
     */
    Button trenchShoot = new Button(() -> switchbox.shoot() && switchbox.guide());
    trenchShoot.whenPressed(
        new RampShooter(-4500)
          .andThen(new Shoot(-4500))
          .alongWith(new SetIndexer(0.45, -4500))
          );
    trenchShoot.whenReleased(() -> {
      shooter.stop();
      hood.stop();
      indexer.stop();
    }, shooter, hood, indexer);
    trenchShoot.whenPressed(new SetHoodAngle(25).andThen(
      new InstantCommand(() -> hood.moveHood(0.02), hood).withTimeout(0.75).andThen(
      new HoldHoodAngle()))
    );

    Button collapseAll = new Button(() -> xboxController.getXButton());
    collapseAll.whenPressed(
        new SetHoodAngle(0).alongWith(new InstantCommand(() -> CPManipulator.setPosition(false), CPManipulator)));

    Button raiseCPM = new Button(() -> xboxController.getBumper(Hand.kRight));
    raiseCPM.whenPressed(() -> CPManipulator.setPosition(true), CPManipulator);

    Button intakeDown = new Button(switchbox::intakeDown);
    intakeDown.whenPressed(intake::down, intake);
    intakeDown.whenPressed(new WaitCommand(0.5).andThen(() -> intake.setOff()));
    intakeDown.whenReleased(intake::up, intake);
    intakeDown.whenReleased(new WaitCommand(0.5).andThen(() -> intake.setOff()));

    Button intaking = new Button(() -> (switchbox.intake() && !(indexer.getNumBalls() > 4 && indexer.getSwitch5())));
    intaking.whileHeld(() -> {
      if (IndexBall.isIndexing()) {
        intake.intake(0.175);
      } else {
        intake.intake();
      }
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
    // rotationControl.whenReleased(() -> CPManipulator.setPosition(false));

    // Button controlPanelSwitch = new Button(() -> switchbox.rotationControl() ||
    // switchbox.positionControl());
    // controlPanelSwitch.whenPressed(() -> CPManipulator.setPosition(true),
    // CPManipulator);
    // controlPanelSwitch.whenReleased(() -> CPManipulator.setPosition(false),
    // CPManipulator);

    Button lampOn = new Button(() -> switchbox.positionControl());
    lampOn.whenPressed(() -> pixy.setLamp((byte) 1, (byte) 1));
    lampOn.whenReleased(() -> pixy.setLamp((byte) 0, (byte) 0));
    lampOn.whileHeld(() -> CPManipulator.update());

    Button positionControl = new Button(() -> switchbox.positionControl() && xboxController.getBButton());

    Button moveIndexer = new Button(() -> (indexer.getSwitch1() && Math.abs(switchbox.getIndexSpeed()) < 0.05 && !switchbox.shoot()));
    moveIndexer.whenPressed(new IndexBall().withTimeout(1));

    Button incrementBall = new Button(() -> indexer.getSwitch5() && Math.abs(switchbox.getIndexSpeed()) < 0.05 && !switchbox.shoot());
    incrementBall.whenPressed(() -> RobotContainer.indexer.incrementBall());
 
    Button moveHoodForBall = new Button(() -> indexer.getNumBalls() >= 4);
    moveHoodForBall.whenPressed(() -> hood.setHood(25), hood);

    Button decreaseBall = new Button(() -> (indexer.getSwitch5() && (switchbox.getIndexSpeed() < -0.05)));
    decreaseBall.whenPressed(indexer::decrementBall);

    Button zeroBalls = new Button(() -> (!switchbox.engagePTO() && switchbox.climb()));
    zeroBalls.whenPressed(indexer::zeroBalls);

    Button resetNavx = new Button(() -> (RobotContainer.xboxController.getStartButton()));
    resetNavx.whenPressed(() -> RobotContainer.swerveDrive.resetNavx(), swerveDrive);

    Button slowSwerve = new Button(() -> xboxController.getTriggerAxis(Hand.kLeft) > 0.65
        && RobotState.isOperatorControl() && RobotState.isEnabled());
    slowSwerve.whenPressed(() -> {
      SwerveDrive.kMaxSpeed = 1;
      SwerveDrive.kMaxAngularSpeed = Math.PI / 2;
    });
    slowSwerve.whenReleased(() -> {
      SwerveDrive.kMaxSpeed = 3.5;
      SwerveDrive.kMaxAngularSpeed = Math.PI;
    });

    Button resetHood = new Button(
        () -> xboxController.getBButton() && xboxController.getAButton() && xboxController.getBumper(Hand.kLeft));
    resetHood.whenPressed(() -> hood.resetEnc());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String trajectoryJSON = "output/TestCurve.wpilib.json";
    Trajectory trajectory = null;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory,
        swerveDrive::getPose,
        swerveDrive.kinematics,
        // Position controllers
        new PIDController(0.85, 0.001, 0.20), // x (forward/backwards)
        new PIDController(0.85, 0.001, 0.20), // y (side to side)
        new ProfiledPIDController(1.75, 0.001, 0.20, new TrapezoidProfile.Constraints(Math.PI, Math.PI)), // theta (rotation)
        swerveDrive::setModuleStates,
        swerveDrive
    );

    return swerveControllerCommand.andThen(() -> swerveDrive.stop());
  }
}
