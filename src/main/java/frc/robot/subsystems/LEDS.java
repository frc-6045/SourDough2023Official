// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDS extends SubsystemBase {
  /** Creates a new LEDS. */
    private AddressableLED m_LED = new AddressableLED(0);
  private AddressableLEDBuffer m_LEDBuffer = new AddressableLEDBuffer(38);


  public LEDS() 
  {
    
    m_LED.setLength(m_LEDBuffer.getLength());
    m_LED.setData(m_LEDBuffer);
    m_LED.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    

  }
  public void startLEDS(){
    m_LED.start();
  }
  public void endLEDS(){
    m_LED.stop();
  }
  
  // public void defaultLEDs()
  // {
  //     // For every pixel
  //     for (var i = 0; i < m_LEDBuffer.getLength(); i++) {
  //       // Calculate the hue - hue is easier for rainbows because the color
  //       // shape is a circle so only one value needs to precess
  //       final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_LEDBuffer.getLength())) % 180;
  //       // Set the value
  //       m_LEDBuffer.setHSV(i, hue, 255, 128);
  //     }
  //     // Increase by to make the rainbow "move"
  //     m_rainbowFirstPixelHue += 3;
  //     // Check bounds
  //     m_rainbowFirstPixelHue %= 180;
    
  // }

  public void SetLEDsPurple()
  { 
    System.out.println("setting to purple");
    for(int i = 0; i < m_LEDBuffer.getLength(); i++)
    {
      m_LEDBuffer.setLED(i, Color.kPurple);
    }
    System.out.println("done");


  }

  public void SetLEDsRed()
  {
    System.out.println("setting to red");
    for(int i = 0; i < m_LEDBuffer.getLength(); i++)
    {
      m_LEDBuffer.setRGB(i, 255, 0, 0);
    }
  }

}
