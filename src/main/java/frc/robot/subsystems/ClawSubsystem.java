package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase{
    
    public final Servo m_leftServo;
    public final Servo m_rightServo;
    
    public ClawSubsystem() {
        m_leftServo = new Servo(ClawConstants.leftServoChannel);
        m_rightServo = new Servo(ClawConstants.rightServoChannel);

    }

    public void ClawPosition(){
        if(ClawConstants.open){
            m_leftServo.setPosition(ClawConstants.openLPosition);
            m_rightServo.setPosition(ClawConstants.openRPosition);
        } else {
            m_leftServo.setPosition(ClawConstants.closeLPosition);
            m_rightServo.setPosition(ClawConstants.closeRPosition);
        }

        ClawConstants.open = !ClawConstants.open;
    }
}
