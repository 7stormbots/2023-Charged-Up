package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalance extends CommandBase{
    
    private final DriveSubsystem m_driveSubsystem;

    public AutoBalance(DriveSubsystem m_driveSubsystem){
        this.m_driveSubsystem = m_driveSubsystem;
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double gyroYaw = m_driveSubsystem.m_gyro.getYaw();
        double gyroPitch = m_driveSubsystem.m_gyro.getPitch();
        double gyroRoll = m_driveSubsystem.m_gyro.getRoll();

        if(gyroPitch >= -5 && gyroPitch <= 5){
            
        }

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
