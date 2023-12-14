package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class LiftCommander extends CommandBase {

    ArmSubsystem m_armSubsystem;

    double threshold;
    boolean looped;
    double target;

    boolean wait;


    public LiftCommander(ArmSubsystem m_armSubsystem, double target) {
        this.m_armSubsystem = m_armSubsystem;
        threshold = 100;
    }

    @Override
    public void initialize() {
        m_armSubsystem.LiftMacro(target, false, true);
    }
    
    @Override
    public void execute() {
        // m_armSubsystem.update();
    }

    @Override
    public boolean isFinished(){
        boolean finished = false;

        if (m_armSubsystem.m_liftLeftMotor.getSelectedSensorPosition() > target - 75 &&
        m_armSubsystem.m_liftLeftMotor.getSelectedSensorPosition() < target + 75) {
            finished = true;
        }
    
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
