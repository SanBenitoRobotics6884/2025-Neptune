// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led_Subsystem extends SubsystemBase {
//hawk tuah
  public Led_Subsystem() {}
AddressableLED m_led = new AddressableLED(9);
AddressableLEDBuffer m_LedBuffer = new AddressableLEDBuffer(60);

  @Override
  public void periodic() {
    m_led.setLength(m_LedBuffer.getLength());
    m_led.setData(m_LedBuffer);
    m_led.start();

    LEDPattern Alliance_red = LEDPattern.solid(Color.kRed);
    LEDPattern Alliance_blue = LEDPattern.solid(Color.kBlue);
    LEDPattern DeepCage = LEDPattern.solid(Color.kGreen);
    LEDPattern elavatorPattern = LEDPattern.solid(Color.kAquamarine );
    LEDPattern algae = LEDPattern.solid(Color.kHotPink);
    LEDPattern coral = LEDPattern.solid(Color.kYellow);
    LEDPattern Pit_mode = LEDPattern.solid(Color.kDarkOrange);

    Alliance_blue.applyTo(m_LedBuffer);
    Alliance_red.applyTo(m_LedBuffer);
    DeepCage.applyTo(m_LedBuffer);
    elavatorPattern.applyTo(m_LedBuffer);
    algae.applyTo(m_LedBuffer);
    coral.applyTo(m_LedBuffer);
    Pit_mode.applyTo(m_LedBuffer);
  }
}
