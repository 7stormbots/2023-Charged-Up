// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

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
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

    public final DriveSubsystem m_robotDrive = new DriveSubsystem();
    public final ArmSubsystem m_armSubsystem = new ArmSubsystem();
    public final ClawSubsystem m_clawSubsystem = new ClawSubsystem();

    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    XboxController m_armController = new XboxController(OIConstants.kArmControllerPort);

    public RobotContainer() {
        configureButtonBindings();

        m_robotDrive.setDefaultCommand(
            new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(
                    m_driverController.getLeftY() * .5,
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                    m_driverController.getLeftX() * .5,
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                    m_driverController.getRightX() * .75,
                    OIConstants.kDriveDeadband),
                false, true),
            m_robotDrive));

        m_armSubsystem.setDefaultCommand(
            new RunCommand(
                () -> m_armSubsystem.ManualControl(m_armController.getLeftY(), 
                m_armController.getRightY())
            )
            
        );
    }


    private void configureButtonBindings() {
        // Swerve Reset
        new JoystickButton(m_driverController, Button.kStart.value)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.setX(),
            m_robotDrive));


        // Lift Controls

        // Higher
        new JoystickButton(m_armController, Button.kRightBumper.value)
                .onTrue((new InstantCommand(
                        () -> m_armSubsystem.LiftMacro(1))));

        // Lower
        new JoystickButton(m_armController, Button.kLeftBumper.value)
                .onTrue(new InstantCommand(
                        () -> m_armSubsystem.LiftMacro(-1)));

        // Extend
        new JoystickButton(m_armController, Button.kA.value)
                .onTrue((new InstantCommand(
                        () -> m_armSubsystem.ExtendMacro())));


        // Claw Controls
        new JoystickButton(m_armController, Button.kA.value)
                .onTrue((new InstantCommand(
                        () -> m_clawSubsystem.ClawPosition(),
                        m_clawSubsystem)));
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