package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.arm.MoveArmDownCommand;
import frc.robot.commands.arm.MoveArmToAmpCommand;
import frc.robot.commands.arm.RaiseArmCommand;
import frc.robot.commands.arm.SetArmToAngleCommand;
import frc.robot.commands.intake.GrabNoteCommand;
import frc.robot.commands.lifter.TogglePistonCommand;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.commands.shooter.ShootNote;
import frc.robot.subsystems.*;
import swervelib.SwerveInputStream;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer2 {

  // The robot's subsystems and commands are defined here...
  private final Drivetrain_Subsystem_Second drivebase = new Drivetrain_Subsystem_Second(new File(Filesystem.getDeployDirectory(),
                                                                       "swerve"));
//   private final CommandXboxController mController =
//       new CommandXboxController(Constants.DRIVER_CONTROLLER_ID);
//   private final CommandXboxController mOperatorController =
//       new CommandXboxController(Constants.OPERATOR_CONTROLLER_ID);

  private final PneumaticsSubsystem mLifterSubsystem = new PneumaticsSubsystem();

  private final ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();

  private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();

  private final ArmSubsystem mArmSubsystem = new ArmSubsystem();

  private final LoaderSubsystem mLoaderSubsystem = new LoaderSubsystem();
  private final LedSubsystem mLedSubsystem = new LedSubsystem();

  private final SendableChooser<Command> mCommandChooser; 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(Constants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

//   Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);



// This is all the code needed to make turning in sim possible and accurate.
  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -driverXbox.getLeftY(),
                                                                   () -> -driverXbox.getLeftX())
                                                               .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                                                               .deadband(Constants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

//   Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer2()
  {
    configureBindings();
    mCommandChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous Chooser", mCommandChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    NamedCommands.registerCommand(
        "grab_note", new GrabNoteCommand(mLoaderSubsystem, mIntakeSubsystem));
    NamedCommands.registerCommand("shoot_note", new ShootNote(mShooterSubsystem, mLoaderSubsystem));
    NamedCommands.registerCommand("move_arm_down", new MoveArmDownCommand(mArmSubsystem));
    NamedCommands.registerCommand(
        "move_arm_to_amp_position", new MoveArmToAmpCommand(mArmSubsystem));
    NamedCommands.registerCommand(
        "move_arm_to_speaker_position",
        new SetArmToAngleCommand(mArmSubsystem, Units.Degrees.of(10.0)));

    Trigger hasNoteTrigger = new Trigger(mLoaderSubsystem::hasNote);
    var inPositionToShootTrigger =
        new Trigger(() -> mArmSubsystem.getArmPosition().isNear(Units.Degrees.of(10.0), 0.25));
    hasNoteTrigger.whileTrue(mLedSubsystem.writeStaticColor(0, 255, 0, 1.0));
    inPositionToShootTrigger.whileTrue(mLedSubsystem.colorFadeCommand(255, 255, 255));
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }
    // hasNoteTrigger.whileFalse(mLedSubsystem.strobeColor(255, 0, 0));

    // driverXbox
    //     .a()
    //     .onTrue(new InstantCommand(drivebase::zeroGyro, drivebase));
    // driverXbox
    //     .rightTrigger(0.75)
    //     .or(
    //         mOperatorController
    //             .a()
    //             .and(mOperatorController.leftBumper().negate())
    //             .and(mOperatorController.rightBumper().negate()))
    //     .whileTrue(new ShootNote(mShooterSubsystem, mLoaderSubsystem));

    // mOperatorController
    //     .y()
    //     .and(mOperatorController.leftBumper())
    //     .whileTrue(new RunLoaderMotor(mLoaderSubsystem, true));
    // mOperatorController
    //     .b()
    //     .and(mOperatorController.leftBumper().negate())
    //     .whileTrue(new RunIntakeMotors(mIntakeSubsystem, false));
    // mOperatorController
    //     .b()
    //     .and(mOperatorController.leftBumper())
    //     .whileTrue(new RunIntakeMotors(mIntakeSubsystem, true));
    // mOperatorController.x().whileTrue(new GrabNoteCommand(mLoaderSubsystem, mIntakeSubsystem));
    // mOperatorController
    //     .y()
    //     .and(mOperatorController.leftBumper().negate())
    //     .whileTrue(new RunLoaderMotor(mLoaderSubsystem, false));

    // mOperatorController
    //     .a()
    //     .and(mOperatorController.rightBumper())
    //     .whileTrue(new RunShooter(mShooterSubsystem, () -> 0.5));
    // mOperatorController
    //     .a()
    //     .and(mOperatorController.leftBumper())
    //     .whileTrue(new RunShooter(mShooterSubsystem, () -> -0.1));

    // mOperatorController
    //     .leftBumper()
    //     .whileTrue(new RaiseArmCommand(mArmSubsystem, () -> mOperatorController.getLeftY() * -0.6));
    // // Original speed -0.45

    // mOperatorController
    //     .povUp()
    //     .whileTrue(new TogglePistonCommand(mLifterSubsystem, true, mArmSubsystem));
    // mOperatorController
    //     .povDown()
    //     .whileTrue(new TogglePistonCommand(mLifterSubsystem, false, mArmSubsystem));

    // // mOperatorController
    // // .start().whileTrue(new BangBangToAngleCommand(mArmSubsystem,
    // // Units.Degrees.of(5.0)));
    // // mOperatorController.start().whileTrue(new SetArmToAngleCommand(mArmSubsystem,
    // // Units.Degrees.of(10.0)));
    // mOperatorController
    //     .rightTrigger(0.5)
    //     .whileTrue(new SetArmToAngleCommand(mArmSubsystem, Units.Degrees.of(10.0)));
    // // mOperatorController
    // //     .leftTrigger(0.5)
    // //     .whileTrue(new SetArmToAngleCommand(mArmSubsystem, Units.Degrees.of(30)));

    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return mCommandChooser.getSelected();
  }
}