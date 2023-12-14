package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ExtendCommander extends CommandBase {

    ArmSubsystem m_armSubsystem;

    double threshold;
    boolean looped;
    double target;

    boolean wait;


    public ExtendCommander(ArmSubsystem m_armSubsystem, double target) {
        this.m_armSubsystem = m_armSubsystem;
        this.target = target;
        threshold = 100;
    }

    @Override
    public void initialize() {
        m_armSubsystem.ExtendMacro((int) target);
    }
    
    @Override
    public void execute() {
        // m_armSubsystem.update();
    }

    @Override
    public boolean isFinished(){
        boolean finished = false;

        if (m_armSubsystem.m_extendEncoder.getPosition() > target - 10 &&
        m_armSubsystem.m_extendEncoder.getPosition() < target + 10) {
            finished = true;
        }
    
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
