// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Hawk Tuah | Led Code
public class Led_Subsystem extends SubsystemBase {
  AddressableLED m_led = new AddressableLED(9);
  AddressableLEDBuffer m_LedBuffer = new AddressableLEDBuffer(60);

    LEDPattern[] patterns = {
      LEDPattern.solid(Color.kRed), //Alliance Red
      LEDPattern.solid(Color.kBlue), // Alliance Blue
      LEDPattern.solid(Color.kGreen), // Deep Cage
      LEDPattern.solid(Color.kAquamarine), //Elevator
      LEDPattern.solid(Color.kHotPink), //Algae
      LEDPattern.solid(Color.kYellow), // Coral
      LEDPattern.solid(Color.kDarkOrange) // PIT MODE
    };

    Optional<Alliance> m_alliance = DriverStation.getAlliance();

  public Led_Subsystem() {
    
  }

  @Override
  public void periodic() {
    m_led.setLength(m_LedBuffer.getLength());    m_led.setData(m_LedBuffer);
    m_led.start();

    patterns[0].applyTo(m_LedBuffer);
    patterns[1].applyTo(m_LedBuffer);
    patterns[2].applyTo(m_LedBuffer);
    patterns[3].applyTo(m_LedBuffer);
    patterns[4].applyTo(m_LedBuffer);
    patterns[5].applyTo(m_LedBuffer);
    patterns[6].applyTo(m_LedBuffer);
    //* MAKE SURE THAT THE SPECIFIC PATTERNS CORRELATE TO THE SITUATION */
  }
}
