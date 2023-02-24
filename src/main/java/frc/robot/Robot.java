// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.autos.TestAuto;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public boolean start;
  public boolean extend;
  public boolean drop;
  public boolean unextend;
  public boolean down;
  public boolean community;
  public boolean follow;

  private RobotContainer m_robotContainer;


  private Command m_autonomousCommand;

  XboxController m_armController = new XboxController(OIConstants.kArmControllerPort);


  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
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
    ClawConstants.open = false;
    ArmConstants.liftState = 1;
    ArmConstants.extendState = 0;
    m_robotContainer.m_clawSubsystem.ClawPosition("Cube");

    start = true;
    extend = false;
    drop = false;
    unextend = false;
    down = false;
    community = false;
    follow = false;

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
  }

  @Override
  public void autonomousPeriodic() {

    if(start){
      m_robotContainer.m_armSubsystem.LiftMacro(ArmConstants.liftHigh, false);
      start = false;
      extend = true;
      drop = false;
      unextend = false;
      down = false;
      community = false;
      follow = false;
    }

    if(extend){
      if(m_robotContainer.m_armSubsystem.m_lift1Encoder.getPosition() >= ArmConstants.liftHigh - 5 && 
      m_robotContainer.m_armSubsystem.m_lift1Encoder.getPosition() <= ArmConstants.liftHigh + 5){
        m_robotContainer.m_armSubsystem.ExtendMacro();
        start = false;
        extend = false;
        drop = true;
        unextend = false;
        down = false;
        community = false;
        follow = false;
      }
    }

    if(drop){
      if(m_robotContainer.m_armSubsystem.m_extendEncoder.getPosition() >= ArmConstants.extendMax - 10 &&
      m_robotContainer.m_armSubsystem.m_extendEncoder.getPosition() <= ArmConstants.extendMax + 10){
      m_robotContainer.m_clawSubsystem.ClawPosition("Cube");
      start = false;
      extend = false;
      drop = false;
      unextend = true;
      down = false;
      community = false;
      follow = false;
      }
    }
   

    if(unextend){
        m_robotContainer.m_armSubsystem.ExtendMacro();
        start = false;
        extend = false;
        drop = false;
        unextend = false;
        down = true;
        community = false;
        follow = false;
    }

    if(down){
      if(m_robotContainer.m_armSubsystem.m_extendEncoder.getPosition() >= ArmConstants.extendMin - 10 &&
      m_robotContainer.m_armSubsystem.m_extendEncoder.getPosition() <= ArmConstants.extendMin + 10){
      m_robotContainer.m_armSubsystem.LiftMacro(ArmConstants.liftIn, true);
      start = false;
      extend = false;
      drop = false;
      unextend = false;
      down = false;
      community = true;
      follow = false;
      }
    }

    if(community) {
      if(m_robotContainer.m_armSubsystem.m_lift1Encoder.getPosition() >= ArmConstants.liftIn - 10 &&
      m_robotContainer.m_armSubsystem.m_lift1Encoder.getPosition() <= ArmConstants.liftIn + 10){
        start = false;
        extend = false;
        drop = false;
        unextend = false;
        down = false;
        community = false;
        follow = true;
      }
    }

    if(follow){
      CommandScheduler.getInstance().run();
    }
    
    m_robotContainer.m_armSubsystem.update();
  }

  @Override
  public void teleopInit() {
    // ArmSubsystem.liftTargetPosition = 0;
    // ArmSubsystem.extendTargetPosition = 0;

    ArmConstants.liftState = 1;
    ArmConstants.extendState = 0;

    m_robotContainer.spdLimit = .85;
    m_robotContainer.turnLimit = .9;

    ClawConstants.open = false;
    m_robotContainer.m_clawSubsystem.ClawPosition("Cone");
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
  }
  }

  @Override
  public void teleopPeriodic() {

    // SmartDashboard.putNumber("AHRS Yaw", m_robotContainer.m_robotDrive.m_gyro.getYaw());
    // SmartDashboard.putBoolean("IMU_Connected", m_robotContainer.m_robotDrive.m_gyro.isConnected());
    // SmartDashboard.putBoolean("IMU_IsCalibrating", m_robotContainer.m_robotDrive.m_gyro.isCalibrating());

    if(Math.abs(m_armController.getLeftY()) > .025 || Math.abs(m_armController.getRightY()) > .025){
      m_robotContainer.m_armSubsystem.ManualControl(-m_armController.getLeftY());
    }

    m_robotContainer.m_armSubsystem.update();

    SmartDashboard.putNumber("Lift Position", m_robotContainer.m_armSubsystem.m_lift1Encoder.getPosition());
    SmartDashboard.putNumber("Extend Position", m_robotContainer.m_armSubsystem.m_extendEncoder.getPosition());
  }
}
