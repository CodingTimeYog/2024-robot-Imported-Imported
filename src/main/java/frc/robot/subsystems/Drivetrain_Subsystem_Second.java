// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.motors.SwerveMotor;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Drivetrain_Subsystem_Second extends SubsystemBase {
  /** Creates a new Drivetrain_Subsystem_Second. */

  private final SwerveDrive mSwerveDrive;

  public double maxSpeed = Units.feetToMeters(14.5);
  private final ShuffleboardLayout mFrontLeftModuleLayout;
  private final ShuffleboardLayout mFrontRightModuleLayout;
  private final ShuffleboardLayout mBackLeftModuleLayout;
  private final ShuffleboardLayout mBackRightModuleLayout;

    /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  private Field2d mField = new Field2d();

  public Drivetrain_Subsystem_Second(File directory) {
        // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    //  In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(16.8);
    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION).
    //  In this case the wheel diameter is 4 inches, which must be converted to meters to get meters/second.
    //  The gear ratio is 6.75 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75);
    System.out.println("\"conversionFactor\": {");
    System.out.println("\t\"angle\": " + angleConversionFactor + ",");
    System.out.println("\t\"drive\": " + driveConversionFactor);
    System.out.println("}");

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    System.out.println("Before try catch");
    try {
      mSwerveDrive = new SwerveParser(directory).createSwerveDrive(maxSpeed);
  // } catch (NoSuchMethodError e) {
  //     System.err.println("NoSuchMethodError: " + e.getMessage());
  //     e.printStackTrace(); // This will show the specific method that is missing.
  //     throw new RuntimeException(e);
  } catch (Exception e) {
      System.err.println("Exception during SwerveDrive creation: " + e.getMessage());
      e.printStackTrace();
      throw new RuntimeException(e);
  }
  
    System.out.println("Out of try catch");
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    tab.add("Field", mField);
    mFrontLeftModuleLayout =
      tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0);
    mFrontRightModuleLayout =
      tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0);
    mBackLeftModuleLayout =
      tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0);
    mBackRightModuleLayout =
      tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0);

    addVoltageToLayout(mFrontLeftModuleLayout, mSwerveDrive.getModules()[0]);
    addVoltageToLayout(mFrontRightModuleLayout, mSwerveDrive.getModules()[1]);
    addVoltageToLayout(mBackLeftModuleLayout, mSwerveDrive.getModules()[2]);
    addVoltageToLayout(mBackRightModuleLayout, mSwerveDrive.getModules()[3]);

    mSwerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    mSwerveDrive.setCosineCompensator(false);
    setupPathPlanner(); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    System.out.println("Set up pathPlanner");
    mSwerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
    System.out.println("Pushed offset values to encoders");
  }



  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = false;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              mSwerveDrive.drive(
                  speedsRobotRelative,
                  mSwerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              mSwerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  public Pose2d getPose()
  {
    return mSwerveDrive.getPose();
  }
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    mSwerveDrive.resetOdometry(initialHolonomicPose);
  }
  public ChassisSpeeds getFieldVelocity()
  {
    return mSwerveDrive.getFieldVelocity();
  }
  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return mSwerveDrive.getRobotVelocity();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    mSwerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void zeroGyro()
  {
    mSwerveDrive.zeroGyro();
    mSwerveDrive.resetOdometry(getPose());
  }
  public Rotation2d getGyroscopeRotation() {
    return mSwerveDrive.getYaw();
  }
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  public Rotation2d getPitch()
  {
    return mSwerveDrive.getPitch();
  }
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return mSwerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return mSwerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }

  
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      mSwerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * mSwerveDrive.getMaximumChassisVelocity(),
                                          Math.pow(translationY.getAsDouble(), 3) * mSwerveDrive.getMaximumChassisVelocity()),
                        Math.pow(angularRotationX.getAsDouble(), 3) * mSwerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }

  private boolean shouldFlip() {
    var alliance = DriverStation.getAlliance();
    return alliance.map(a -> a.equals(Alliance.Red)).orElse(false);
  }
  private static DoubleSupplier makeDriveMotorVolatageSupplier(SwerveModule pModule) {
    // Use a lambda to return the double as a DoubleSupplier
    SwerveMotor driveMotor = pModule.getDriveMotor();
    return () -> driveMotor.getVoltage(); // Wrap in lambda to match DoubleSupplier type
}


private static DoubleSupplier makeAngleMotorTemperatureSupplier(SwerveModule pModule) {
  // Use a lambda to return the double as a DoubleSupplier
  SwerveMotor driveMotor = pModule.getAngleMotor();
  return () -> driveMotor.getVoltage(); // Wrap in lambda to match DoubleSupplier type
}

  private static void addVoltageToLayout(ShuffleboardLayout pLayout, swervelib.SwerveModule swerveModule) {
    pLayout.addNumber("Drive Motor Temperature", makeDriveMotorVolatageSupplier(swerveModule));
    pLayout.addNumber("Steer Motor Temperature", makeAngleMotorTemperatureSupplier(swerveModule));
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    mSwerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }
  public void drive(ChassisSpeeds velocity)
  {
    mSwerveDrive.drive(velocity);
  }
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
      double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
      // Make the robot move
      driveFieldOriented(mSwerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      mSwerveDrive.getOdometryHeading().getRadians(),
                                                                      mSwerveDrive.getMaximumChassisVelocity()));
    });
  }
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    mSwerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      mSwerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public SwerveDriveKinematics getKinematics()
  {
    return mSwerveDrive.kinematics;
  }

  public void postTrajectory(Trajectory trajectory)
  {
    mSwerveDrive.postTrajectory(trajectory);
  }
  public void setMotorBrake(boolean brake)
  {
    mSwerveDrive.setMotorIdleMode(brake);
  }
  private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }
  public void zeroGyroWithAlliance()
  {
    if (isRedAlliance())
    {
      zeroGyro();
      //Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else
    {
      zeroGyro();
    }
  }

  public SwerveDrive getSwerveDrive()
  {
    return mSwerveDrive;
  }
  @Override
  public void periodic() {
    // System.out.println(getSwerveDrive().getModules()[0].getAbsolutePosition());
    // System.out.println(getSwerveDrive().getModules()[1].getAbsolutePosition());
    // System.out.println(getSwerveDrive().getModules()[2].getAbsolutePosition());
    // System.out.println(getSwerveDrive().getModules()[3].getAbsolutePosition());

    SmartDashboard.putNumber("Backleft relative position", getSwerveDrive().getModules()[0].getAbsolutePosition());
    SmartDashboard.putNumber("Backright relative position", getSwerveDrive().getModules()[1].getAbsolutePosition());
    SmartDashboard.putNumber("frontleft relative position", getSwerveDrive().getModules()[2].getAbsolutePosition());
    SmartDashboard.putNumber("Backleft relative position", getSwerveDrive().getModules()[2].getAbsolutePosition());

  }
}
