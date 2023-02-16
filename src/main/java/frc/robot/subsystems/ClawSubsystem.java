package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase{
    
    public final Servo m_leftServo;
    public final Servo m_rightServo;

    public int position;
    
    public ClawSubsystem() {
        m_leftServo = new Servo(0);
        m_rightServo = new Servo(1);

    }


    public void ClawPosition(boolean open){
        if(open){
            m_leftServo.setPosition(ClawConstants.openPosition);
            m_rightServo.setPosition(-ClawConstants.openPosition);
        } else {
            m_leftServo.setPosition(ClawConstants.closePosition);
            m_rightServo.setPosition(-ClawConstants.closePosition);
        }
    }
}
