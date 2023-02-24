package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TestAuto extends SequentialCommandGroup {
    public void auto(DriveSubsystem swerveSubsystem) {
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(1.71, 1.12, Rotation2d.fromDegrees(0)),
                        new Pose2d(3.16, 0.87, Rotation2d.fromDegrees(0)),
                        new Pose2d(5.84, 0.83, Rotation2d.fromDegrees(0))),
                config);


        PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);

        ProfiledPIDController thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, 
                Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                testTrajectory,
                swerveSubsystem::getPose,
                AutoConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        addCommands(
            new InstantCommand(() -> swerveSubsystem.resetOdometry(testTrajectory.getInitialPose())),
            swerveControllerCommand);


        swerveSubsystem.periodic();
    }
}
