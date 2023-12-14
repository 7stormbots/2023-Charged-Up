package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase{
    public final CANSparkMax m_intakeMotor;
    public final RelativeEncoder m_intakeEncoder;

    public final Servo m_leftServo;
    public final Servo m_rightServo;

    double lastPosition = 0;
    String lastType = null;

    Timer clawTimer;
    
    public ClawSubsystem() {
        m_leftServo = new Servo(ClawConstants.leftServoChannel);
        m_rightServo = new Servo(ClawConstants.rightServoChannel);
        m_intakeMotor = new CANSparkMax(ClawConstants.intakeMotorCanId, MotorType.kBrushless);

        m_intakeEncoder = m_intakeMotor.getEncoder();
        
        m_intakeMotor.setIdleMode(ClawConstants.kIntakeMotorIdleMode);

        clawTimer = new Timer();
    }

    public void ClawPosition(String type){
        this.lastType = type;

        if (type.equals("Cone")) {
            if(ClawConstants.open) {
                m_intakeMotor.set(-0.15);
                m_leftServo.setPosition(0);
                m_rightServo.set(0);
            } else {
                m_intakeMotor.set(0.3);
                m_leftServo.setPosition(.5);
                m_rightServo.set(.5);
            }
        } else {
            if(ClawConstants.open) {
                m_intakeMotor.set(.15);
                m_leftServo.setPosition(1);
                m_rightServo.set(1);
            } else {
                m_intakeMotor.set(-.15);
                m_leftServo.setPosition(1);
                m_rightServo.set(1);
            }
        }
        clawTimer = new Timer();
        clawTimer.start();
        ClawConstants.open = !ClawConstants.open;            
    }

    public void update() {
        

        if (ClawConstants.open && clawTimer.get() > .5) {
            if (m_intakeEncoder.getVelocity() < 2000) {
                m_intakeMotor.set(0.06);
            }
        }


        SmartDashboard.putNumber("Claw Timer", clawTimer.get());
        SmartDashboard.putNumber("IntakeMotor Velocity", m_intakeEncoder.getVelocity());
    }
}
