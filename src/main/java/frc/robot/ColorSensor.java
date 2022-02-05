// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

/** Add your docs here. */
public class ColorSensor extends Base{
    

    int blueCount, redCount, yellowCount, greenCount;
    boolean colorCheck = false;
    boolean changedColor = true;
    String colorString = "Unknown";
    String pastColor = "Unknown";
    String firstColor = "Unknown";

    /**
     * Change the I2C port below to match the connection of your color sensor
     */
    private final I2C.Port i2cPort = I2C.Port.kMXP;  //kOnboard;
  
    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a 
     * parameter. The device will be automatically initialized with default 
     * parameters.
     */
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  
    /**
     * A Rev Color Match object is used to register and detect known colors. This can 
     * be calibrated ahead of time or during operation.
     * 
     * This object uses a simple euclidian distance to estimate the closest match
     * with given confidence range.
     */
    private final ColorMatch m_colorMatcher = new ColorMatch();
  

    ShuffleboardTab colorSensorTab = Shuffleboard.getTab("ColorSensor");
    NetworkTableEntry ntRedColor = colorSensorTab.add("Red", 0).getEntry();
    NetworkTableEntry ntBlueColor = colorSensorTab.add("Blue", 0).getEntry();
    NetworkTableEntry ntConfidence = colorSensorTab.add("Confidence", 0).getEntry();
    NetworkTableEntry ntDetected = colorSensorTab.add("Detected Color", 0).getEntry();
    NetworkTableEntry ntBlueCount = colorSensorTab.add("blue count", 0).getEntry();
    NetworkTableEntry ntRedCount = colorSensorTab.add("red count", 0).getEntry();
    NetworkTableEntry ntGreenCount = colorSensorTab.add("green count", 0).getEntry();

    public static boolean ballDetected = false;

    public static boolean redBallDetected = false;
    public static boolean blueBallDetected = false;

    //Robot Init
    public void init() {
      m_colorMatcher.addColorMatch(Constants.kBlueBallColor);
      m_colorMatcher.addColorMatch(Constants.kRedBallColor);
    }
  
    //Robot Periodic
    public void checkColor() {
      /**
       * The method GetColor() returns a normalized color value from the sensor and can be
       * useful if outputting the color to an RGB LED or similar. To
       * read the raw color, use GetRawColor().
       * 
       * The color sensor works best when within a few inches from an object in
       * well lit conditions (the built in LED is a big help here!). The farther
       * an object is the more light from the surroundings will bleed into the 
       * measurements and make it difficult to accurately determine its color.
       */
      Color detectedColor = m_colorSensor.getColor();
  
      /**
       * Run the color match algorithm on our detected color
       */
      
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
  
      if(match.confidence > 0.8) {
        ballDetected = true;
      } else {
        ballDetected = false;
      }
      
      redBallDetected = false;
      blueBallDetected = false;

      if(detectedColor.red > 0.8) {
        redBallDetected = true;
      } else if(detectedColor.blue > 0.8) {
        blueBallDetected = true;
      } else {

      }

      /**
       * Open Smart Dashboard or Shuffleboard to see the color detected by the 
       * sensor.
       */
      ntRedColor.setDouble(detectedColor.red);
      ntBlueColor.setDouble(detectedColor.blue);
      ntConfidence.setDouble(match.confidence);
      ntDetected.setString(colorString);
      ntBlueCount.setDouble(blueCount);
      ntRedCount.setDouble(redCount);
      ntGreenCount.setDouble(greenCount);

    }

}
