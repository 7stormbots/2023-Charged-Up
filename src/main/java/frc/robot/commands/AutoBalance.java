package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalance extends CommandBase {
    
    private final DriveSubsystem m_driveSubsystem;

    double gyroPitch;
    double gyroRoll;

    boolean switchMode = false;
    boolean exit = false;

    public AutoBalance(DriveSubsystem m_driveSubsystem){
        this.m_driveSubsystem = m_driveSubsystem;
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        gyroPitch = m_driveSubsystem.m_gyro.getPitch();

        if(!(Math.abs(gyroPitch) >= 6) && !switchMode) {
            m_driveSubsystem.autoBalanceToggle = false;
            m_driveSubsystem.drive(.15, 0, 0, false, true);
        } else if (Math.abs(gyroPitch) >= 6 && !switchMode) {
            switchMode = true;
        } else if (switchMode) {
            switchMode = true;
            m_driveSubsystem.autoBalanceToggle = true;
            m_driveSubsystem.drive(0, 0, 0, false, true);
            
            // if(!(Math.abs(gyroPitch) >= 6)) {
            //     exit = true;
            // }
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(0, 0, 0, true, true);
        m_driveSubsystem.autoBalanceToggle = false;
        LimelightConstants.limelightToggle = false;
    }
}
