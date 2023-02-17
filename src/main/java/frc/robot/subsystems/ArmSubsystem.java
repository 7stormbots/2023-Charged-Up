package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    public final CANSparkMax m_liftMotor;
    public final CANSparkMax m_extendMotor;
    public final RelativeEncoder m_liftEncoder;
    public final RelativeEncoder m_extendEncoder;

    public TrapezoidProfile.Constraints m_liftConstraints = new Constraints(
        ArmConstants.liftMaxVel, ArmConstants.liftMaxAcc);
    public ProfiledPIDController m_liftController = new ProfiledPIDController(
        ArmConstants.liftKp, ArmConstants.liftKi, ArmConstants.liftKd, m_liftConstraints);


    public TrapezoidProfile.Constraints m_extendConstraints = new Constraints(
        ArmConstants.extendMaxVel, ArmConstants.extendMaxAcc);
    public ProfiledPIDController m_extendController = new ProfiledPIDController(
        ArmConstants.extendKp, ArmConstants.extendKi, ArmConstants.extendKd, m_extendConstraints);


    


    // public static int liftTargetPosition;
    // public static int extendTargetPosition;

    public ArmSubsystem() {
        m_liftMotor = new CANSparkMax(ArmConstants.liftMotorCanId, MotorType.kBrushless);
        m_extendMotor = new CANSparkMax(ArmConstants.extendMotorCanId, MotorType.kBrushless);

        m_liftEncoder = m_liftMotor.getEncoder();
        m_extendEncoder = m_extendMotor.getEncoder();

        // m_liftController.setGoal(0);
        // m_extendController.setGoal(0);
    }

    public void LiftControl(double leftTrigger, double rightTrigger) {
        // int power = (int) (800 * (leftTrigger - rightTrigger));
        // liftTargetPosition = (int) (m_liftEncoder.getPosition() + power);

        // if(liftTargetPosition > ArmConstants.maxLift){
        //     liftTargetPosition = ArmConstants.maxLift;
        // } else if (liftTargetPosition < ArmConstants.minLift){
        //     liftTargetPosition = ArmConstants.minLift; // a
        // }
    }

    public void LiftMacro(double target){
        m_liftController.setGoal(target);
        System.out.println("Lift Macro Ran");
        System.out.println("Lift Target: " + target);
    }

    public void ExtendMacro(double target){
        m_extendController.setGoal(target);
    }

    // public void ExtendControl(double rightJoyX) {
    //     double power = rightJoyX;
    //     m_extendMotor.set(extendPower);
    // }

    public void update() {
        m_liftMotor.set(m_liftController.calculate(m_liftEncoder.getPosition()));
        m_extendMotor.set(m_extendController.calculate(m_extendEncoder.getPosition()));
    }
}
