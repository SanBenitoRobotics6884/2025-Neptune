package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.HardwareConfigs;

import static frc.robot.Constants.Elevator.*;

public class ElevatorSubsystem extends SubsystemBase {
 /* Left and Right Motors */
  private TalonFX m_leftMotor;
  //private TalonFX m_rightMotor;

  private PIDController pidController;
  //private Follower m_follower = new Follower(LEFT_MOTOR_ID, true);
  private double speed = 0.5;
  private double THRESHOLD = 5.0;
  private double LOWER_LIMIT = Constants.Elevator.MIN;
  private double UPPER_LIMIT = Constants.Elevator.MAX;
  private double lastAction = System.currentTimeMillis();

  double eMotorPosition = 0.0;

  public ElevatorSubsystem() {

    HardwareConfigs hardwareConfigs = new HardwareConfigs();
    //m_leftMotor = new TalonFX(LEFT_MOTOR_ID, "Galigma");
    m_leftMotor = new TalonFX(RIGHT_MOTOR_ID, "Galigma");
    // Load your TalonFX drive config (PID, current limit, etc.)
    var m_leftMotorConfigurator = m_leftMotor.getConfigurator();
    //var m_rightMotorConfigurator = m_rightMotor.getConfigurator();
    m_leftMotorConfigurator.apply(hardwareConfigs.getElevatorConfig(false));
    //m_rightMotorConfigurator.apply(hardwareConfigs.getElevatorConfig(true));
    m_leftMotorConfigurator.setPosition(0.0);
    //m_rightMotorConfigurator.setPosition(0.0);
    //m_rightMotor.setControl(m_follower);

    eMotorPosition = 0.0;
  }

  private void setEMotorPosition(double val){
    eMotorPosition = val;
    if (eMotorPosition > UPPER_LIMIT) {
      eMotorPosition = UPPER_LIMIT;
    }
    else if (eMotorPosition < LOWER_LIMIT) {
      eMotorPosition = LOWER_LIMIT;
    }
  }

  private void setEMotorVoltage(double val, boolean isDirectInput){
    // Checks current motor position in rotations (angles/360)
    double position = m_leftMotor.getPosition().getValue().in(Degrees)/360;

    if (isDirectInput) {
      if(val > 0 && position > UPPER_LIMIT){
        this.stop();
        return;
      } if(val < 0 && position < LOWER_LIMIT){
        this.stop();
        return;
      }
      m_leftMotor.setVoltage(val);
    } else {
      // Need to check for button presses
    }
   
  }
  public void gotolevel(double val){
    double position = m_leftMotor.getPosition().getValue().in(Degrees)/360;

    if(position == val){
      this.stop();
    } else if  (position - val > 0){
      this.setEMotorVoltage(9, ENABLECURRENTLIMIT);
    }else{
      this.setEMotorVoltage(-9, ENABLECURRENTLIMIT);
    }

  }


  public void extend(double val){
    /*if (System.currentTimeMillis() - lastAction < 100){ // every 100ms
      return;
    }
    lastAction = System.currentTimeMillis();
    setEMotorPosition(eMotorPosition + val);
    m_leftMotor.set(speed);*/
    // m_rightMotor.set(-speed);

    this.setEMotorVoltage(9, true);
  }

  

  public void retract(double val){
    /*if (System.currentTimeMillis() - lastAction < 100){ // every 100ms
      return;
    }
    lastAction = System.currentTimeMillis();
    setEMotorPosition(eMotorPosition - val);
    m_leftMotor.set(-speed);*/
    //m_rightMotor.set(speed);

    this.setEMotorVoltage(-9, true);
  }

  public void stop(){
    //setEMotorPosition(m_leftMotor.getPosition().getValue().in(Degrees)/360);
    m_leftMotor.setVoltage(0.2);
  }

  public void cycle(){
    if (m_leftMotor.getPosition().getValue().in(Degrees)/360 == LOWER_LIMIT) {
      m_leftMotor.setPosition(UPPER_LIMIT);
      m_leftMotor.set(speed);
    } else if (m_leftMotor.getPosition().getValue().in(Degrees)/360 == LOWER_LIMIT) {
      m_leftMotor.setPosition(LOWER_LIMIT);
      m_leftMotor.set(-speed);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator actual", m_leftMotor.getPosition().getValue().in(Degrees)/360);
    SmartDashboard.putNumber("Elevator target", eMotorPosition);
    //if(Math.abs(eMotorPosition - m_leftMotor.getPosition().getValue().in(Degrees)/360) > THRESHOLD)
    //  m_leftMotor.setPosition(eMotorPosition);
  }
}