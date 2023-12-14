// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.ClawCommander;
import frc.robot.commands.ExtendCommander;
import frc.robot.commands.LiftCommander;
import frc.robot.commands.LimelightDistance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RobotContainer {

    public final DriveSubsystem m_robotDrive = new DriveSubsystem();
    public final ClawSubsystem m_clawSubsystem = new ClawSubsystem();
    public final ArmSubsystem m_armSubsystem = new ArmSubsystem(m_clawSubsystem);
    public static final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
    public static final LimelightDistance m_limelightDistance = new LimelightDistance();


    PS4Controller m_driverController = new PS4Controller(OIConstants.kDriverControllerPort);
    PS4Controller m_armController = new PS4Controller(OIConstants.kArmControllerPort);

    public double spdLimit = DriveConstants.spdLimitFast;
    public double turnLimit = DriveConstants.turnLimitFast;

    public boolean fieldCentric;


    public RobotContainer() {
        configureButtonBindings();

        m_robotDrive.setDefaultCommand(
            new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(
                    Math.pow(m_driverController.getLeftY(), 2) * Math.signum(m_driverController.getLeftY()) * spdLimit,
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                    Math.pow(m_driverController.getLeftX(), 2) * Math.signum(m_driverController.getLeftX()) * spdLimit,
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                    Math.pow(m_driverController.getRightX(), 2) * Math.signum(m_driverController.getRightX()) * turnLimit,
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
            .whileTrue(new InstantCommand(
                () -> changeMode(!fieldCentric)
            ));

        new JoystickButton(m_driverController, Button.kCircle.value)
            .whileTrue(new InstantCommand(
                () -> m_robotDrive.BalanceToggle()
            ));


        new JoystickButton(m_driverController, Button.kTriangle.value)
            .whileTrue(new RunCommand(
                () -> resetIMU()
            ));

        // Swerve Speed Limit
        new POVButton(m_driverController, 0)
            .onTrue(new InstantCommand(
                () -> updateLimits(DriveConstants.spdLimitFast, DriveConstants.turnLimitFast)
                ));
                
        new POVButton(m_driverController, 180)
            .onTrue(new InstantCommand(
                () -> updateLimits(DriveConstants.spdLimitSlow, DriveConstants.spdLimitSlow)
            ));

        new JoystickButton(m_driverController, Button.kR1.value)
            .whileTrue(new InstantCommand(
                () -> limelightToggleFunc()
            ));

            new JoystickButton(m_driverController, Button.kSquare.value)
            .whileTrue(new InstantCommand(
                () -> WallAlignToggle()
            ));


        // Lift Controls

        // Arm Macros

        new JoystickButton(m_armController, Button.kShare.value)
                .onTrue((new InstantCommand(
                    () -> m_armSubsystem.LiftMacro(5200, false, false))));

        new JoystickButton(m_armController, Button.kOptions.value)
                .onTrue((new InstantCommand(
                    () -> m_armSubsystem.LiftMacro(ArmConstants.liftDropIntake, false, false))));

        new POVButton(m_armController, 0)
                .onTrue((new InstantCommand(
                        () -> m_armSubsystem.LiftMacro(ArmConstants.liftHigh, false, false))));

        new POVButton(m_armController, 270)
                .onTrue((new InstantCommand(
                        () -> m_armSubsystem.LiftMacro(ArmConstants.liftMid, false, false))));

        new POVButton(m_armController, 90)
                .onTrue((new InstantCommand(
                        () -> m_armSubsystem.LiftMacro(ArmConstants.liftLow, false, false))));
        
        new POVButton(m_armController, 180)
                .onTrue((new InstantCommand(
                        () -> m_armSubsystem.LiftMacro(ArmConstants.liftIn, true, false))));


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

    

    public void limelightToggleFunc(){
        LimelightConstants.limelightToggle = !LimelightConstants.limelightToggle;
    }

    public void WallAlignToggle() {
        m_robotDrive.wallAlignToggle = !m_robotDrive.wallAlignToggle;
    }
                       
    public void resetIMU(){
        m_robotDrive.m_gyro.zeroYaw();
    }

    public void changeMode(boolean centric){
        this.fieldCentric = centric;
    }

    public void updateLimits(double spdLimit, double turnLimit){
        this.spdLimit = spdLimit;
        this.turnLimit = turnLimit;
    }


    public Command forwardAutonomousCommand() {

        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        Trajectory traj1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.43, 4.83, Rotation2d.fromDegrees(0)),
                List.of(
                ),
                new Pose2d(4.54, 4.69, Rotation2d.fromDegrees(0)),
                config); 

        PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);

        ProfiledPIDController thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, 
                Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand SwerveCommand1 = new SwerveControllerCommand(
            traj1,
            m_robotDrive::getPose,
            AutoConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

        return new SequentialCommandGroup(
            new InstantCommand(() -> m_robotDrive.resetOdometry(traj1.getInitialPose())),
            SwerveCommand1,
            new InstantCommand(() -> m_robotDrive.stopModules())
        );
    }

    public Command leftAutonomousCommand() {

        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        Trajectory traj1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.41, 4.81, Rotation2d.fromDegrees(180)),
                List.of(
                ),
                new Pose2d(1.99, 4.81, Rotation2d.fromDegrees(180)),
                config); 
        
        Trajectory traj2P1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(1.99, 4.81, Rotation2d.fromDegrees(180)),
                List.of(
                ),
                new Pose2d(3.47, 4.67, Rotation2d.fromDegrees(180)),
                config); 

        Trajectory traj2P2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(3.47, 4.67, Rotation2d.fromDegrees(180)),
                List.of(
                ),
                new Pose2d(6.25, 4.55, Rotation2d.fromDegrees(0)),
                config); 
        
        
        Trajectory traj3 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(6.25, 4.55, Rotation2d.fromDegrees(0)),
                List.of(
                    new Translation2d(2.69, 4.76),
                    new Translation2d(2.18, 4.18)
                ),
                new Pose2d(2.43, 3.79, Rotation2d.fromDegrees(180)),
                config); 

        Trajectory traj4 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.43, 3.79, Rotation2d.fromDegrees(180)),
                List.of(
                ),
                new Pose2d(1.99, 3.8, Rotation2d.fromDegrees(180)),
                config);

        Trajectory traj5 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(1.99, 3.8, Rotation2d.fromDegrees(180)),
                List.of(
                    new Translation2d(2.23, 3)
                ),
                new Pose2d(3.5, 2.82, Rotation2d.fromDegrees(180)),
                config); 

        Trajectory traj2 = traj2P1.concatenate(traj2P2);

        PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);

        ProfiledPIDController thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, 
                Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand SwerveCommand1 = new SwerveControllerCommand(
                traj1,
                m_robotDrive::getPose,
                AutoConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);

        SwerveControllerCommand SwerveCommand2 = new SwerveControllerCommand(
                traj2,
                m_robotDrive::getPose,
                AutoConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);

        SwerveControllerCommand SwerveCommand3 = new SwerveControllerCommand(
                traj3,
                m_robotDrive::getPose,
                AutoConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);

        SwerveControllerCommand SwerveCommand4 = new SwerveControllerCommand(
                traj4,
                m_robotDrive::getPose,
                AutoConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);

        m_armSubsystem.firstRun = false;

        return new SequentialCommandGroup(
            // new InstantCommand(() -> m_robotDrive.resetOdometry(traj1.getInitialPose())),
            // new ParallelCommandGroup(
            // new ClawCommander(m_clawSubsystem, false, "Cone"),
            new LiftCommander(m_armSubsystem, ArmConstants.liftHigh)
            // new ClawCommander(m_clawSubsystem, true, "Cone")
            // new RunCommand(() -> new LiftCommander(m_armSubsystem, ArmConstants.liftHigh)),
            // SwerveCommand1,
            // new RunCommand(() -> m_clawSubsystem.ClawPosition("Cone")),
            // new ParallelCommandGroup(
            //     new LiftCommander(m_armSubsystem, ArmConstants.liftIn),
            //     new ClawCommander(m_clawSubsystem, true, "Cube")),
            // SwerveCommand2,
            // new ParallelCommandGroup(
            //     new ClawCommander(m_clawSubsystem, false, "Cone"),
            //     new ExtendCommander(m_armSubsystem, ArmConstants.extendMin)),
            // SwerveCommand3,
            // new LiftCommander(m_armSubsystem, ArmConstants.liftHigh),
            // SwerveCommand4,
            // new ParallelCommandGroup(
            //     new AutoBalance(m_robotDrive),
            //     new LiftCommander(m_armSubsystem, ArmConstants.liftLow)
            // ),
            // new InstantCommand(() -> m_robotDrive.stopModules())
        );
    }

    public Command middleAutonomousCommand() {
        return new SequentialCommandGroup(
            new RunCommand(() -> new AutoBalance(m_robotDrive))
        );
    }

    // public Command middleAutonomousCommand() {
    //     TrajectoryConfig config = new TrajectoryConfig(
    //             AutoConstants.kMaxSpeedMetersPerSecond,
    //             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //             .setKinematics(DriveConstants.kDriveKinematics);

    //     Trajectory traj1 = TrajectoryGenerator.generateTrajectory(
    //             new Pose2d(1.93, 2.82, Rotation2d.fromDegrees(180)),
    //             List.of(
    //             ),
    //             new Pose2d(3.2, 2.82, Rotation2d.fromDegrees(180)),
    //             config); 


    //     PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
    //     PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);

    //     ProfiledPIDController thetaController = new ProfiledPIDController(
    //             Constants.AutoConstants.kPThetaController, 0, 0, 
    //             Constants.AutoConstants.kThetaControllerConstraints);
    //     thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //     SwerveControllerCommand SwerveCommand1 = new SwerveControllerCommand(
    //             traj1,
    //             m_robotDrive::getPose,
    //             AutoConstants.kDriveKinematics,
    //             xController,
    //             yController,
    //             thetaController,
    //             m_robotDrive::setModuleStates,
    //             m_robotDrive);

    //     return new SequentialCommandGroup(
    //         // new InstantCommand(() -> m_ExtendCommander.ExtendTarget(ArmConstants.extendMin, true)),
    //         // new InstantCommand(() -> m_LiftCommander.LiftTarget(ArmConstants.liftMid, true)),
    //         // new InstantCommand(() -> m_ExtendCommander.ExtendTarget(48, true)),
    //         // new InstantCommand(() -> m_clawSubsystem.ClawPosition("Cone")),
    //         // new InstantCommand(() -> m_ExtendCommander.ExtendTarget(ArmConstants.extendMin, true)),
    //         // new InstantCommand(() -> m_LiftCommander.LiftTarget(ArmConstants.liftIn, true)),
    //         new InstantCommand(() -> m_robotDrive.resetOdometry(traj1.getInitialPose())),
    //         SwerveCommand1,
    //         new RunCommand(() -> m_robotDrive.AutoBalance()),
    //         new InstantCommand(() -> m_robotDrive.stopModules())
    //     );
    // }


    public Command rightAutonomousCommand() {
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        // Trajectory traj1 = TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(2.35, 4.87, Rotation2d.fromDegrees(180)),
        //         List.of(
        //         ),
        //         new Pose2d(1.98, 4.87, Rotation2d.fromDegrees(180)),
        //         config); 

        // Trajectory traj2 = TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(1.98, 4.87, Rotation2d.fromDegrees(180)),
        //             List.of(
        //             ),
        //             new Pose2d(3.41, 4.67, Rotation2d.fromDegrees(180)),
        //             config); 

        // Trajectory traj3 = TrajectoryGenerator.generateTrajectory(
        //                 new Pose2d(3.41, 4.67, Rotation2d.fromDegrees(180)),
        //                     List.of(
        //                     ),
        //                     new Pose2d(6.25, 4.53, Rotation2d.fromDegrees(0)),
        //                     config); 


        // Trajectory Traj2 = traj2.concatenate(traj3);


        PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);

        ProfiledPIDController thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, 
                Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // SwerveControllerCommand SwerveCommand1 = new SwerveControllerCommand(
        //         traj1,
        //         m_robotDrive::getPose,
        //         AutoConstants.kDriveKinematics,
        //         xController,
        //         yController,
        //         thetaController,
        //         m_robotDrive::setModuleStates,
        //         m_robotDrive);

                // SwerveControllerCommand SwerveCommand2 = new SwerveControllerCommand(
                //     Traj2,
                //     m_robotDrive::getPose,
                //     AutoConstants.kDriveKinematics,
                //     xController,
                //     yController,
                //     thetaController,
                //     m_robotDrive::setModuleStates,
                //     m_robotDrive);

                //     SwerveControllerCommand SwerveCommand3 = new SwerveControllerCommand(
                //     traj3,
                //     m_robotDrive::getPose,
                //     AutoConstants.kDriveKinematics,
                //     xController,
                //     yController,
                //     thetaController,
                //     m_robotDrive::setModuleStates,
                //     m_robotDrive);

        return new SequentialCommandGroup(
            new RunCommand(() -> m_robotDrive.resetOdometry(new Pose2d(2.45, 0.56, Rotation2d.fromDegrees(180)))),
            // new RunCommand(() -> m_LiftCommander.LiftTarget(3000, true)),
            // new RunCommand(() -> m_ExtendCommander.ExtendTarget(ArmConstants.extendMax, true)),
            // new RunCommand(() -> m_LiftCommander.LiftTarget(ArmConstants.liftHigh, true)),
            // SwerveCommand1,
            // new InstantCommand(() -> m_clawSubsystem.ClawPosition("Cone")),
            // new RunCommand(() -> m_ExtendCommander.ExtendTarget(ArmConstants.extendMin, true)),
            // new ParallelCommandGroup(
            //     addCommands(
            //         m_LiftCommander.LiftTarget(3000, true)
            // )),
            // new ParallelCommandGroup(m_LiftCommander.LiftTarget(3000, true)),
            // SwerveCommand2,
            // SwerveCommand3,
            new InstantCommand(() -> m_robotDrive.stopModules())
        );
    }
}


// Mahi Stinks