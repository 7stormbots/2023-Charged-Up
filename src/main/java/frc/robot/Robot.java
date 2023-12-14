// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.ClawCommander;
import frc.robot.commands.DriveCommand;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {


  private RobotContainer m_robotContainer;


  private Command m_leftAutonomous;
  private Command m_middleAutonomous;
  private Command m_rightAutonomous;

  private Command m_testAutonomous;

  XboxController m_armController = new XboxController(OIConstants.kArmControllerPort);


  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

   public static final String defaultAuto = "Default Autonomous";
   public static final String leftAuto = "Left Autonomous";
   public static final String middleAuto = "Middle Autonomous";
   public static final String rightAuto = "Right Autonomous";
   public String autoSelected;
   public static final SendableChooser<String> autoChooser = new SendableChooser<>();

   boolean firstRun;


   boolean moveArm = true;
   boolean firstArmToggle = false;
   boolean driveForward = false;
   boolean open = false;
   boolean unextend = false;
   boolean firstUnextendToggle = false;
   boolean resetArm = false;
   boolean switchMode = false;

   boolean balance = false;

   boolean communityTimerToggle = false;
   boolean communityTimerFirstToggle = false;
   boolean community = false;
   Timer autoTimer;

   boolean auto = false;


   boolean done = false;

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture(1);
    m_robotContainer.m_robotDrive.m_gyro.calibrate();

    autoChooser.setDefaultOption("No Autonomous (Default)", defaultAuto);
    
    autoChooser.addOption("Left Autonomous", leftAuto);
    autoChooser.addOption("Middle Autonomous", middleAuto);
    autoChooser.addOption("Right Autonomous", rightAuto);
    SmartDashboard.putData("Autos", autoChooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    ArmConstants.liftState = 1;
    ArmConstants.extendState = 1;

    // m_robotContainer.m_robotDrive.autoBalanceToggle = false;


    m_robotContainer.m_armSubsystem.auto = false;

    autoSelected = autoChooser.getSelected();

    m_robotContainer.m_robotDrive.zeroHeading();

    // m_rightAutonomous = m_robotContainer.rightAutonomousCommand();

    // if (m_rightAutonomous != null) {
    //   m_rightAutonomous.schedule();
    // }
    ClawConstants.open = false;
    m_robotContainer.m_clawSubsystem.ClawPosition("Cone");
  

    m_robotContainer.m_armSubsystem.firstRun = true;

    firstRun = false;

    moveArm = false;
    firstArmToggle = true;
    driveForward = false;
    open = false;
    unextend = false;
    resetArm = false;
    switchMode = false;
    done = false;

    balance = true;
    // communityTimerToggle = true;
    // communityTimerFirstToggle = true;
    // community = true;

    // m_robotContainer.m_robotDrive.autoBalanceToggle = false;
    // LimelightConstants.limelightToggle = false;
    // m_robotContainer.fieldCentric = false;



    auto = true;


    // m_leftAutonomous = m_robotContainer.forwardAutonomousCommand();
      m_robotContainer.m_clawSubsystem.m_intakeMotor.set(.075);
  }

  @Override
  public void autonomousPeriodic() {


    // switch (autoSelected) {
    //   case leftAuto:    
    //   m_leftAutonomous = m_robotContainer.forwardAutonomousCommand();
    //     break;

    //     case middleAuto:       
          // m_middleAutonomous = m_robotContainer.leftAutonomousCommand();

          // m_robotContainer.m_clawSubsystem.update();
          // m_robotContainer.m_armSubsystem.update();
 
    //     break;

    //  case rightAuto:       
    //    m_rightAutonomous = m_robotContainer.rightAutonomousCommand();

    //    break;

    //  case defaultAuto:
    //  default: 
    //       break;
    // }

     

    // if(auto) {
      // if (m_leftAutonomous != null) {
      //   m_leftAutonomous.schedule();
      // }
      // auto = false;
    // }
       



      // if(communityTimerToggle) {
      //   autoTimer = new Timer();
      //   autoTimer.start();
      //   communityTimerToggle = false;
      // }
      // w
      // if(community && )



      // m_robotContainer.m_robotDrive.drive(-.1, 0, 0, false, false);

      // SmartDashboard.putNumber("Auto Timer", autoTimer.get());

      // if(community && communityTimerFirstToggle && autoTimer.get() < 5) {
      //   m_robotContainer.m_robotDrive.drive(-.1, 0, 0, false, true);
      // } else if (communityTimerFirstToggle && autoTimer.get() > 5) {
      //   communityTimerFirstToggle = false;
      //   m_robotContainer.m_robotDrive.drive(0, 0, 0, false, true);
      // } else {
      //   if(community && !communityTimerFirstToggle && autoTimer.get() < 10) {
      //     m_robotContainer.m_robotDrive.drive(.1, 0, 0, false, true);
      //   } else {
      //     m_robotContainer.m_robotDrive.drive(0, 0, 0, false, true);
      //   }
      // }

      // if (balance) {

      // }

       if (balance) {
        double gyroPitch = m_robotContainer.m_robotDrive.m_gyro.getPitch();
        if(!(Math.abs(gyroPitch) >= 12) && !switchMode) {
          m_robotContainer.m_robotDrive.autoBalanceToggle = false;
          m_robotContainer.m_robotDrive.drive(.25, 0, 0, false, true);
        } else if (Math.abs(gyroPitch) >= 12 && !switchMode) {
          switchMode = true;
        } else if (switchMode) {
          m_robotContainer.m_robotDrive.autoBalanceToggle = true;
          m_robotContainer.m_robotDrive.drive(0, 0, 0, false, true);
        }
      }
      

      m_robotContainer.m_clawSubsystem.update();

  }

  @Override
  public void teleopInit() {

    if(m_leftAutonomous != null) {
      m_leftAutonomous.cancel();
    }
    if(m_middleAutonomous != null) {
      m_middleAutonomous.cancel();
    }
    if(m_rightAutonomous != null) {
      m_rightAutonomous.cancel();
    }

    m_robotContainer.m_armSubsystem.auto = false;
    // ArmSubsystem.liftTargetPosition = 0;
    // ArmSubsystem.extendTargetPosition = 0;

    // m_robotContainer.m_armSubsystem.m_extendController.setGoal(ArmConstants.extendMin);


    m_robotContainer.m_robotDrive.autoBalanceToggle = false;


    m_robotContainer.m_armSubsystem.m_liftLeftMotor.set(TalonSRXControlMode.Position, 0);
    ArmConstants.liftTarget = 0;

    ArmConstants.liftState = 1;
    ArmConstants.extendState = 1;
    

    m_robotContainer.fieldCentric = true;
    LimelightConstants.limelightToggle = false;

    m_robotContainer.m_armSubsystem.unextend = false;
    ClawConstants.open = false;
    m_robotContainer.m_clawSubsystem.ClawPosition("Cone");

    // m_robotContainer.m_armSubsystem.manualOffset = -7;
    // m_robotContainer.m_armSubsystem.offset = 0;

    // m_robotContainer.m_armSubsystem.homeExtend(true);
    m_robotContainer.m_armSubsystem.firstRun = true;
    // m_robotContainer.m_armSubsystem.manualOffset = 5;
    m_robotContainer.m_armSubsystem.m_liftController.setGoal(m_robotContainer.m_armSubsystem.m_liftLeftMotor.getSelectedSensorPosition() + 100);
    m_robotContainer.m_armSubsystem.m_extendController.setGoal(ArmConstants.extendMin);
    m_robotContainer.m_armSubsystem.manualOffset = 0;
  }

  @Override
  public void teleopPeriodic() {



    if(Math.abs(m_robotContainer.m_armController.getLeftY()) > .1 || Math.abs(m_robotContainer.m_armController.getRightY()) > .1){
      m_robotContainer.m_armSubsystem.ManualControl(-m_armController.getLeftY(), -m_armController.getRightY());
    }

    if (m_armController.getLeftBumper()) {
      m_robotContainer.m_armSubsystem.ManualWristLeft();
    } else if (m_armController.getRightBumper()) {
      m_robotContainer.m_armSubsystem.ManualWristRight();
    }

  

    m_robotContainer.m_armSubsystem.update();
    m_robotContainer.m_clawSubsystem.update();


    SmartDashboard.putNumber("AHRS Yaw (DEG)", m_robotContainer.m_robotDrive.m_gyro.getYaw());
    SmartDashboard.putNumber("AHRS Pitch (DEG)", m_robotContainer.m_robotDrive.m_gyro.getPitch());
    SmartDashboard.putNumber("AHRS Roll (DEG)", m_robotContainer.m_robotDrive.m_gyro.getRoll());

    // SmartDashboard.putNumber("Lift Position", m_robotContainer.m_armSubsystem.m_liftLeftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Lift Power", m_robotContainer.m_armSubsystem.m_liftLeftMotor.get());
    SmartDashboard.putNumber("Lift PID Position Error", 
    m_robotContainer.m_armSubsystem.m_liftLeftMotor.get() * 2_000);

    // Lift Target -> ArmSubsystem

    SmartDashboard.putNumber("Intake Position", m_robotContainer.m_clawSubsystem.m_intakeEncoder.getCountsPerRevolution());


    SmartDashboard.putNumber("Extend Position", m_robotContainer.m_armSubsystem.m_extendEncoder.getPosition());
    SmartDashboard.putNumber("Extend Velocity", m_robotContainer.m_armSubsystem.m_extendEncoder.getVelocity());
    SmartDashboard.putNumber("Extend Power", m_robotContainer.m_armSubsystem.m_extendMotor.get());
    // Extend Target -> ArmSubsystem
    
    SmartDashboard.putNumber("Wrist Position", m_robotContainer.m_armSubsystem.m_wristEncoder.getPosition());
    SmartDashboard.putNumber("Wrist Power", m_robotContainer.m_armSubsystem.m_wristMotor.get());
    // Wrist Target -> ArmSubsystem
    // Wrist Target (DEG) -> ArmSubsystem
    // Wrist Manual Offset -> ArmSubsystem

    SmartDashboard.putNumber("IMU -> Wheel Rotation", Math.toDegrees(m_robotContainer.m_robotDrive.inputTranslationDir));
    SmartDashboard.putNumber("IMU -> Wheel Power", m_robotContainer.m_robotDrive.inputTranslationMag);
    SmartDashboard.putBoolean("Auto Balance", m_robotContainer.m_robotDrive.autoBalanceToggle);

    SmartDashboard.putBoolean("Vardhan Stinky?", true);

    SmartDashboard.putNumber("Intake Position", m_robotContainer.m_clawSubsystem.m_intakeEncoder.getVelocity());


  }
}
