// package frc.robot.autos;
// import frc.robot.Constants;
// import frc.robot.subsystems.DriveSubsystem;

// import java.util.List;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;


// public class LeftBlueAuto extends SequentialCommandGroup {
//     public void testingAuto(DriveSubsystem swerveSubsystem){
//         TrajectoryConfig config = new TrajectoryConfig(
//                 Constants.AutoConstants.kMaxSpeedMetersPerSecond, //Sets Max speed of the bot in auton
//                 Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared) //Sets Max acceleration of the bot in auton
//                     .setKinematics(Constants.Swerve.SwerveKinematics); //Gets all the kinematics info for swerve

//         // Basic trajectory using the traj generator tool built into WPILib
//         Trajectory testingTrajectory = TrajectoryGenerator.generateTrajectory(
//                 // Sets the start direction
//                 new Pose2d(0, 0, new Rotation2d(0)),
//                 // Should go in a straight line
//                 (TrajectoryConfig) List.of(
//                     new Pose2d(2.3, 4.61, Rotation2d.fromDegrees(0)),
//                     new Pose2d(6.85, 4.56, Rotation2d.fromDegrees(0)),
//                     new Pose2d(2.37, 4.61, Rotation2d.fromDegrees(0)),
//                     new Pose2d(3.86, 2.77, Rotation2d.fromDegrees(0)), //2nd point 2 meters ahead of where we started
//                      //Sets end pose 3 meters ahead of starting point
//                 config));

//         PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
//         PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
//         ProfiledPIDController thetaController = new ProfiledPIDController(
//                 Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
//         thetaController.enableContinuousInput(-Math.PI, Math.PI);

//         SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//                 testingTrajectory,
//                 swerveSubsystem::getPose,
//                 Constants.Swerve.SwerveDriveKinematics,
//                 xController,
//                 yController,
//                 thetaController,
//                 swerveSubsystem::setModuleStates,
//                 swerveSubsystem);


//         addCommands(
//             new InstantCommand(() -> swerveSubsystem.resetOdometry(testingTrajectory.getInitialPose())),
//             swerveControllerCommand);
//     }
// }

