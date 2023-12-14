package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
    
    private final DriveSubsystem m_driveSubsystem;

    

    public DriveCommand(DriveSubsystem m_driveSubsystem){
        this.m_driveSubsystem = m_driveSubsystem;
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        m_driveSubsystem.drive(-.1, 0, 0, false, false);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
