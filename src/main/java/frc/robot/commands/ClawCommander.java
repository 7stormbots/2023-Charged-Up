package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class ClawCommander extends CommandBase {

    ClawSubsystem m_clawSubsystem;

    double threshold;
    boolean looped;
    double target;
    boolean open;
    String type;

    boolean wait;


    public ClawCommander(ClawSubsystem m_clawSubsystem, boolean open, String type) {
        this.m_clawSubsystem = m_clawSubsystem;
        this.target = target;
        this.open = open;
        this.type = type;
        threshold = 100;
    }

    @Override
    public void initialize() {
        ClawConstants.open = open;
        m_clawSubsystem.ClawPosition(type);
    }
    
    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
