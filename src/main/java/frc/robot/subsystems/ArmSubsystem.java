package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;

public class ArmSubsystem extends SubsystemBase {

    public final WPI_TalonSRX m_liftLeftMotor;
    public final WPI_TalonSRX m_liftRightMotor;
    public final CANSparkMax m_extendMotor;
    public final CANSparkMax m_wristMotor;

    public final RelativeEncoder m_extendEncoder;
    public final RelativeEncoder m_wristEncoder;

    public TrapezoidProfile.Constraints m_liftConstraints = new Constraints(
        ArmConstants.liftMaxVel, ArmConstants.liftMaxAcc);
    public ProfiledPIDController m_liftController = new ProfiledPIDController(
        ArmConstants.liftKp, ArmConstants.liftKi, ArmConstants.liftKd, m_liftConstraints);

    public TrapezoidProfile.Constraints m_extendConstraints = new Constraints(
        ArmConstants.extendMaxVel, ArmConstants.extendMaxAcc);
    public ProfiledPIDController m_extendController = new ProfiledPIDController(
        ArmConstants.extendKp, ArmConstants.extendKi, ArmConstants.extendKd, m_extendConstraints);

        public TrapezoidProfile.Constraints m_wristConstraints = new Constraints(
        ArmConstants.wristMaxVel, ArmConstants.wristMaxAcc);
    public ProfiledPIDController m_wristController = new ProfiledPIDController(
        ArmConstants.wristKp, ArmConstants.wristKi, ArmConstants.wristKd, m_wristConstraints);

    ClawSubsystem m_clawSubsystem;


    public boolean unextend;
    public int offset;
    public int manualOffset;
    public boolean auto;

    public boolean resetArm;

    public double extendOffset;

    boolean liftIn;
    boolean liftLow;
    boolean liftMid;

    boolean liftHigh;
    boolean firstStage = true;
    public boolean firstRun = true;


    public ArmSubsystem(ClawSubsystem m_clawSubsystem) {
        this.m_clawSubsystem = m_clawSubsystem;
        m_liftLeftMotor = new WPI_TalonSRX(ArmConstants.liftLeftMotorCanId);
        m_liftRightMotor = new WPI_TalonSRX(ArmConstants.liftRightMotorCanId);
        m_extendMotor = new CANSparkMax(ArmConstants.extendMotorCanId, MotorType.kBrushless);
        m_wristMotor = new CANSparkMax(ArmConstants.wristMotorCanId, MotorType.kBrushless);

        m_liftLeftMotor.configFactoryDefault();
        m_liftRightMotor.configFactoryDefault();
        m_extendMotor.restoreFactoryDefaults();
        m_wristMotor.restoreFactoryDefaults();

        m_extendEncoder = m_extendMotor.getEncoder();
        m_wristEncoder = m_wristMotor.getEncoder();


        m_extendMotor.setIdleMode(ArmConstants.kExtendMotorIdleMode);
        m_wristMotor.setIdleMode(ArmConstants.kWristMotorIdleMode);

        m_liftLeftMotor.configPeakCurrentLimit(ArmConstants.kLiftMotorCurrentLimit);
        m_liftRightMotor.configPeakCurrentLimit(ArmConstants.kLiftMotorCurrentLimit);
        m_extendMotor.setSmartCurrentLimit(ArmConstants.kExtendMotorCurrentLimit);
        m_wristMotor.setSmartCurrentLimit(ArmConstants.kWristMotorCurrentLimit);

        m_liftLeftMotor.config_kP(0, 0);
        m_liftLeftMotor.config_kI(0, .00005);
        m_liftLeftMotor.config_kD(0, 0);
        m_liftLeftMotor.config_kF(0, 0);
        m_liftLeftMotor.config_IntegralZone(0, 4000);

        m_liftRightMotor.config_kP(0, 0);
        m_liftRightMotor.config_kI(0, .00005);
        m_liftRightMotor.config_kD(0, 0);
        m_liftRightMotor.config_kF(0, 0);
        m_liftRightMotor.config_IntegralZone(0, 4000);

        m_liftController.setIntegratorRange(-200, 200);


        m_liftLeftMotor.setInverted(true);
        m_wristMotor.setInverted(false);

        // m_liftLeftMotor.burnFlash();
        // m_liftRightMotor.burnFlash();
        m_extendMotor.burnFlash();
        m_wristMotor.burnFlash();
    }

    public void ManualControl(double liftPower, double extendPower) {

        double liftTarget;
        double extendTarget;

        liftHigh = false;
        liftMid = false;
        liftLow = false;
        liftIn = false;
        auto = false;

        if(Math.abs(liftPower) >= .1){
            liftTarget = m_liftLeftMotor.getSelectedSensorPosition() + (liftPower * ArmConstants.maxLiftInterval);
            if(liftTarget > ArmConstants.liftMax){
                liftTarget = ArmConstants.liftMax;
            }
            if(liftTarget < ArmConstants.liftMin){
                liftTarget = ArmConstants.liftMin;
            }
            m_liftController.setGoal(liftTarget);
            ArmConstants.liftTarget = liftTarget;
            SmartDashboard.putNumber("Lift Target", liftTarget);
            
        }
        if(Math.abs(extendPower) >= .1){
            extendTarget = m_extendEncoder.getPosition() + (extendPower * ArmConstants.maxExtendInterval);  
            if(extendTarget < ArmConstants.extendMin){
                extendTarget = ArmConstants.extendMin;
            }
            if(extendTarget > ArmConstants.extendMax){
                extendTarget = ArmConstants.extendMax;
            }
            m_extendController.setGoal(extendTarget);
            SmartDashboard.putNumber("Extend Target", extendTarget);  
        }

        firstRun = false;
    }

    public void ManualWristLeft () {
        manualOffset = manualOffset + 1;
    }
    
    public void ManualWristRight () {
        manualOffset = manualOffset - 1;
    }

    public void LiftMacro(double target, boolean in, boolean auto){
        if(target == ArmConstants.liftDropIntake) {
            m_extendController.setGoal(ArmConstants.extendMin);
            m_liftController.setGoal(target);
            ClawConstants.open = false;
            m_clawSubsystem.ClawPosition("Cone");
            manualOffset = 30;
        } else if (target == ArmConstants.liftHigh) {
            liftHigh = true;
            liftMid = false;
            liftLow = false;
            liftIn = false;
        } else if (target == ArmConstants.liftMid) {
            manualOffset = 17;
            liftHigh = false;
            liftMid = true;
            liftLow = false;
            liftIn = false;
        } else if (target == ArmConstants.liftLow && !auto) {
            manualOffset = 6;
            liftHigh = false;
            liftMid = false;
            liftLow = true;
            liftIn = false;
        } else if (target == ArmConstants.liftIn) {
            manualOffset = 20;
            liftHigh = false;
            liftMid = false;
            liftLow = false;
            liftIn = true;
        }

        if(liftHigh) {
            firstStage = true;
            m_liftController.setGoal(ArmConstants.liftLow + 1000);
        } else if (liftMid) {
            m_liftController.setGoal(target);
        } else if (liftLow) {
            m_liftController.setGoal(target);
        } else if (liftIn) {
            m_liftController.setGoal(ArmConstants.liftLow);
            m_extendController.setGoal(ArmConstants.extendMin);
        }

        firstRun = false;

        SmartDashboard.putNumber("Lift Target", target);
    }

    public void ExtendMacro(int extend){
        double target;
        if (extend == 1) {
            target = ArmConstants.extendMax;
        } else {
            target = ArmConstants.extendMin;
        }
        
        m_extendController.setGoal(target);
        SmartDashboard.putNumber("Extend Target", target);
       
    }

    public void WristControl(double deg){
        double degToTicks = (deg / 360) * 70;

        m_wristController.setGoal(degToTicks);
        SmartDashboard.putNumber("Wrist Target", degToTicks);   
        SmartDashboard.putNumber("Wrist Target Deg", deg);
    }

    public void WristControlT(double ticks){
        m_wristController.setGoal(ticks);
        SmartDashboard.putNumber("Wrist Target", ticks);
    }

    
    public static double liftToDeg;

    public void update() {

        liftToDeg = -(m_liftLeftMotor.getSelectedSensorPosition() / 65.88);

        if(!firstRun) {
            if (m_liftLeftMotor.getSelectedSensorPosition() <= 1500) {
                WristControl(0);
            } else {
                WristControl(liftToDeg + manualOffset);
            }
        } else {
            WristControlT(MathUtil.clamp(1 + manualOffset, -5, 5));
        }

        
        if(liftHigh) {
            if(firstStage) {
                if(m_liftLeftMotor.getSelectedSensorPosition() >= ArmConstants.liftLow + 1000 - 75 &&
                m_liftLeftMotor.getSelectedSensorPosition() <= ArmConstants.liftLow + 1000 + 75) {
                    firstStage = false;
                }
            } else if (!firstStage) {
                m_extendController.setGoal(ArmConstants.extendMax);
                if(m_extendEncoder.getPosition() >= ArmConstants.extendMax - 25) {
                    m_liftController.setGoal(ArmConstants.liftHigh);
                    ClawConstants.open = false;
                    m_clawSubsystem.ClawPosition("Cone");
                    manualOffset = 10;
                    liftHigh = false;
                    firstStage = false;
                }
            }
        }

        if(!auto) {

        if(liftLow) {
            if(m_liftLeftMotor.getSelectedSensorPosition() >= ArmConstants.liftLow - 75 &&
            m_liftLeftMotor.getSelectedSensorPosition() <= ArmConstants.liftLow + 75) {
                m_extendController.setGoal(200);
                liftLow = false;
            }
        }
        
        
        if(liftIn) {
            if(m_extendEncoder.getPosition() <= ArmConstants.extendMin + 15) {
                m_liftController.setGoal(ArmConstants.liftIn);
                liftIn = false;
            }
        }
        }
        

        if(auto) {
            if(m_extendEncoder.getPosition() <= ArmConstants.extendMin + 25) {
                m_liftController.setGoal(ArmConstants.liftLow);
                liftLow = false;
            }
        }

        SmartDashboard.putNumber("Lift Position", m_liftLeftMotor.getSelectedSensorPosition());

        m_liftLeftMotor.set(m_liftController.calculate(m_liftLeftMotor.getSelectedSensorPosition()) / 2_000);
        m_liftRightMotor.follow(m_liftLeftMotor);

        m_extendMotor.set(m_extendController.calculate(m_extendEncoder.getPosition()));
        m_wristMotor.set(m_wristController.calculate(m_wristEncoder.getPosition()));
    }
}
    