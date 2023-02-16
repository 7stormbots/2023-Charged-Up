// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

public class RobotContainer {

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final ClawSubsystem m_ClawSubsystem = new ClawSubsystem();

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_armController = new XboxController(OIConstants.kArmControllerPort);

  // PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example Path", new
  // PathConstraints(4, 3));

  // // This will load the file "FullAuto.path" and generate it with a max
  // velocity of 4 m/s and a max acceleration of 3 m/s^2
  // // for every path in the group
  // ArrayList<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("New
  // Path", new PathConstraints(4, 3));

  // // This is just an example event map. It would be better to have a constant,
  // global event map
  // // in your code that will be used by all path following commands.
  // HashMap<String, Command> eventMap = new HashMap<>();
  // eventMap.@put("marker1".new PrintCommand("Passed marker 1"));
  // eventMap.put("intakeDown".new IntakeDown());

  // // Create the AutoBuilder. This only needs to be created once when robot code
  // starts, not every time you want to create an auto command. A good place to
  // put this is in RobotContainer along with your subsystems.
  // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
  // DriveSubsystem::getPose, // Pose2d supplier
  // DriveSubsystem::resetPose, // Pose2d consumer, used to reset odometry at the
  // beginning of auto
  // DriveSubsystem.SwerveDriveKinematics, // SwerveDriveKinematics
  // new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation
  // error (used to create the X and Y PID controllers)
  // new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation
  // error (used to create the rotation controller)
  // DriveSubsystem::setModuleStates, // Module states consumer used to output to
  // the drive subsystem
  // eventMap,
  // true; // Should the path be automatically mirrored depending on alliance
  // color. Optional, defaults to true
  // // The drive subsystem. Used to properly set the requirements of path
  // following commands
  // );

  // Command fullAuto = autoBuilder.fullAuto(pathGroup);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                false, true),
            m_robotDrive));

    // m_ArmSubsystem.setDefaultCommand(

    // new RunCommand(
    // () -> m_ArmSubsystem.LiftControl(
    // m_armController.getLeftTriggerAxis(),
    // m_armController.getRightTriggerAxis()),
    // m_ArmSubsystem));

  }

  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));


    new JoystickButton(m_armController, Button.kCross.value)
        .onTrue((new RunCommand(
            () -> m_ArmSubsystem.LiftMacro(ArmConstants.liftMax),
            m_ArmSubsystem)));

    new JoystickButton(m_armController, Button.kCircle.value)
        .onTrue(new RunCommand(
            () -> m_ArmSubsystem.LiftMacro(ArmConstants.liftMin),
            m_ArmSubsystem)); 


    new JoystickButton(m_armController, Button.kTriangle.value)
        .onTrue((new RunCommand(
            () -> m_ArmSubsystem.ExtendMacro(ArmConstants.extendMax),
            m_ArmSubsystem)));

    new JoystickButton(m_armController, Button.kSquare.value)
        .onTrue(new RunCommand(
            () -> m_ArmSubsystem.ExtendMacro(ArmConstants.extendMin),
            m_ArmSubsystem));
  }

  public Command getAutonomousCommand() {

    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)

        .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}