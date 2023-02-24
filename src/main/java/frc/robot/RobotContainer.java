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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RobotContainer {

    public final DriveSubsystem m_robotDrive = new DriveSubsystem();
    public final ArmSubsystem m_armSubsystem = new ArmSubsystem();
    public final ClawSubsystem m_clawSubsystem = new ClawSubsystem();

    PS4Controller m_driverController = new PS4Controller(OIConstants.kDriverControllerPort);
    PS4Controller m_armController = new PS4Controller(OIConstants.kArmControllerPort);

    public double spdLimit;
    public double turnLimit;

    public boolean fieldCentric;

    public RobotContainer() {
        configureButtonBindings();

        m_robotDrive.setDefaultCommand(
            new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(
                    m_driverController.getLeftY() * spdLimit,
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                    m_driverController.getLeftX() * spdLimit,
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                    m_driverController.getRightX() * turnLimit,
                    OIConstants.kDriveDeadband),
                fieldCentric, true),
            m_robotDrive));
    }


    private void configureButtonBindings() {
        // Swerve Reset
        new JoystickButton(m_driverController, Button.kOptions.value)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.setX(),
            m_robotDrive));

        new JoystickButton(m_driverController, Button.kCross.value)
            .whileTrue(new RunCommand(
                () -> changeMode(!fieldCentric)
            ));


        // Swerve Speed Limit
        new JoystickButton(m_driverController, 2)
            .onTrue(new InstantCommand(
                () -> updateLimits(.85, .9)
                ))
            .onFalse(new InstantCommand(
                () -> updateLimits(.3, .25)
            ));


        // Lift Controls

        // Arm Macros
        new POVButton(m_armController, 0)
                .onTrue((new InstantCommand(
                        () -> m_armSubsystem.LiftMacro(ArmConstants.liftHigh, false))));

        new POVButton(m_armController, 270)
                .onTrue((new InstantCommand(
                        () -> m_armSubsystem.LiftMacro(ArmConstants.liftMid, false))));


        new POVButton(m_armController, 90)
                .onTrue((new InstantCommand(
                        () -> m_armSubsystem.LiftMacro(ArmConstants.liftLow, false))));
        
        new POVButton(m_armController, 180)
                .onTrue((new InstantCommand(
                        () -> m_armSubsystem.LiftMacro(ArmConstants.liftIn, true))));


        // Extend
        new JoystickButton(m_armController, Button.kTriangle.value)
                .onTrue((new InstantCommand(
                        () -> m_armSubsystem.ExtendMacro())));


        // Claw Cone Control
        new JoystickButton(m_armController, Button.kSquare.value)
                .onTrue((new InstantCommand(
                        () -> m_clawSubsystem.ClawPosition("Cone"),
                        m_clawSubsystem)));

        // Claw Cube Control
        new JoystickButton(m_armController, Button.kCircle.value)
                .onTrue((new InstantCommand(
                        () -> m_clawSubsystem.ClawPosition("Cube"),
                        m_clawSubsystem)));
    }

    public void changeMode(boolean centric){
        this.fieldCentric = centric;
    }

    public void updateLimits(double spdLimit, double turnLimit){
        this.spdLimit = spdLimit;
        this.turnLimit = turnLimit;
    }

    public Command getAutonomousCommand() {
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(1.66, 1.12, Rotation2d.fromDegrees(0)),
                List.of(
                        new Translation2d(-3.36, .75)
                ),
                new Pose2d(-5.99, 0.78, Rotation2d.fromDegrees(90)),
                config);


        PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);

        ProfiledPIDController thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, 
                Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                testTrajectory,
                m_robotDrive::getPose,
                AutoConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);

        return new SequentialCommandGroup(
            new InstantCommand(() -> m_robotDrive.resetOdometry(testTrajectory.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> m_robotDrive.stopModules())
        );
    }
}