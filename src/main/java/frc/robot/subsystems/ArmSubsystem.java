package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    public final CANSparkMax m_lift1Motor;
    // public final CANSparkMax m_lift2Motor;
    public final CANSparkMax m_extendMotor;
    public final RelativeEncoder m_lift1Encoder;
    public final RelativeEncoder m_extendEncoder;

    public TrapezoidProfile.Constraints m_liftConstraints = new Constraints(
        ArmConstants.liftMaxVel, ArmConstants.liftMaxAcc);
    public ProfiledPIDController m_liftController = new ProfiledPIDController(
        ArmConstants.liftKp, ArmConstants.liftKi, ArmConstants.liftKd, m_liftConstraints);

    public TrapezoidProfile.Constraints m_extendConstraints = new Constraints(
        ArmConstants.extendMaxVel, ArmConstants.extendMaxAcc);
    public ProfiledPIDController m_extendController = new ProfiledPIDController(
        ArmConstants.extendKp, ArmConstants.extendKi, ArmConstants.extendKd, m_extendConstraints);


    public ArmSubsystem() {
        m_lift1Motor = new CANSparkMax(ArmConstants.lift2MotorCanId, MotorType.kBrushless);
        // m_lift2Motor = new CANSparkMax(ArmConstants.lift2MotorCanId, MotorType.kBrushless);
        m_extendMotor = new CANSparkMax(ArmConstants.extendMotorCanId, MotorType.kBrushless);

        m_lift1Motor.restoreFactoryDefaults();
        m_extendMotor.restoreFactoryDefaults();

        m_lift1Encoder = m_lift1Motor.getEncoder();
        m_extendEncoder = m_extendMotor.getEncoder();

        m_extendMotor.setIdleMode(ArmConstants.kExtendMotorIdleMode);

        m_lift1Motor.setSmartCurrentLimit(ArmConstants.kLiftMotorCurrentLimit);
        m_extendMotor.setSmartCurrentLimit(ArmConstants.kExtendMotorCurrentLimit);

        m_lift1Motor.burnFlash();
        m_extendMotor.burnFlash();
    }

    public void ManualControl(double liftPower) {
            double liftTarget = m_lift1Encoder.getPosition() + (liftPower * ArmConstants.maxLiftInterval);
            // double extendTarget = m_extendEncoder.getPosition() + (extendPower * ArmConstants.maxExtendInterval);

            if(liftTarget > ArmConstants.liftMax){
                liftTarget = ArmConstants.liftMax;
            }
            if(liftTarget < ArmConstants.liftMin){
                liftTarget = ArmConstants.liftMax;
            }

            // if(extendTarget > ArmConstants.extendMax){
            //     extendTarget = ArmConstants.extendMax;
            // }
            // if(extendTarget < ArmConstants.extendMin){
            //     extendTarget = ArmConstants.extendMax;
            // }
            m_liftController.setGoal(liftTarget);
            // m_extendController.setGoal(extendTarget);
            SmartDashboard.putNumber("Lift Target", liftTarget);
            // SmartDashboard.putNumber("Extend Target", extendTarget);
    }

    public void LiftMacro(double target, boolean in){
        if(in){
            ArmConstants.extendState = 0;
            m_extendController.setGoal(ArmConstants.extendMin);
        }
        m_liftController.setGoal(target);
        SmartDashboard.putNumber("Target", target);
    }

    public void ExtendMacro(){
        double target;

        if(ArmConstants.extendState == 0){
            ArmConstants.extendState = 1;
            target = ArmConstants.extendMax;
        } else {
            ArmConstants.extendState = 0;
            target = ArmConstants.extendMin;
        }
        
        m_extendController.setGoal(target);
        SmartDashboard.putNumber("Target", target);
    }

    public void update() {

        m_lift1Motor.set(m_liftController.calculate(m_lift1Encoder.getPosition()));
        // m_lift2Motor.set(m_liftController.calculate(m_lift1Encoder.getPosition()));
        m_extendMotor.set(m_extendController.calculate(m_extendEncoder.getPosition()));

        SmartDashboard.putNumber("Calculated Lift 1", m_liftController.calculate(m_lift1Encoder.getPosition()));
        // SmartDashboard.putNumber("Calculated Lift 2", -m_liftController.calculate(m_lift1Encoder.getPosition()));
        SmartDashboard.putNumber("Calculated Extend", m_extendController.calculate(m_extendEncoder.getPosition()));
    }
}







    // public void LiftHoming(){
    //     SparkMaxPIDController m_liftHomePID = m_liftMotor.getPIDController();


    //     for (int i = 0; i < ArmConstants.liftHomeTime; i++) {
    //             m_liftHomePID.setReference(-1, CANSparkMax.ControlType.kCurrent);
    //     }
    //     ArmConstants.liftOffset = m_extendEncoder.getPosition();
    //     m_liftHomePID.setReference(ArmConstants.liftOffset, CANSparkMax.ControlType.kPosition);
    //     SmartDashboard.putNumber("Extend Offset", ArmConstants.liftOffset);
    // }

    // public void ExtendHoming(){
    //     // SparkMaxPIDController m_extendHomePID = m_extendMotor.getPIDController();


    //     for (int i = 0; i < ArmConstants.extendHomeTime; i++) {
    //         m_extendMotor.setVoltage(-1);
    //     }
    //     ArmConstants.extendOffset = m_extendEncoder.getPosition();
    //     // m_extendHomePID.setReference(ArmConstants.extendOffset, CANSparkMax.ControlType.kPosition);
    //     SmartDashboard.putNumber("Extend Offset", ArmConstants.extendOffset);
    //     m_extendMotor.setVoltage(0);
    // }