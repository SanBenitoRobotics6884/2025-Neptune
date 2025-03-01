// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Hawk Tuah | Led Code
public class Led_Subsystem extends SubsystemBase {
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_LedBuffer;
  
    Optional<Alliance> m_alliance = DriverStation.getAlliance();

  public Led_Subsystem() {
    m_led = new AddressableLED(Constants.LEDs.LED_BUFFER_LENGTH);
    m_LedBuffer = new AddressableLEDBuffer(60);

    m_led.setLength(m_LedBuffer.getLength());   
    m_led.setData(m_LedBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {

    for (int i = 0; i == 6; i++) {
      Constants.LEDs.LED_PATTERNS[i].applyTo(m_LedBuffer);
    }
  }
}
