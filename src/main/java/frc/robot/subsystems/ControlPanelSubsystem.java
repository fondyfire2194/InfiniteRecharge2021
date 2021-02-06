/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ControlPanelConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;

public class ControlPanelSubsystem extends SubsystemBase {
   /**
    * Creates a new ControlPanel.
    */
   /**
    * Change the I2C port below to match the connection of your color sensor
    */
   private final I2C.Port i2cPort = I2C.Port.kOnboard;
   // private final TalonSRX turnMotor = new TalonSRX(ControlPanelConstants.TURN_MOTOR);
   /**
    * A Rev Color Sensor V3 object is constructed with an I2C port as a parameter.
    * The device will be automatically initialized with default parameters.
    */
   private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

   /**
    * A Rev Color Match object is used to register and detect known colors. This
    * can be calibrated ahead of time or during operation.
    * 
    * This object uses a simple euclidian distance to estimate the closest match
    * with given confidence range.
    */
   private final ColorMatch m_colorMatcher = new ColorMatch();
   private Color detectedColor;

   private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
   private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
   private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
   private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

   ShuffleboardTab colors;
   String colorString;
   public int colorNumber;
   public int colorNumberFiltered;
   private int loopCount;

   private int filterNumber = 3;
   private int redCount;
   private int yellowCount;
   private int blueCount;
   private int greenCount;

   public int gameColorNumber;
   private SuppliedValueWidget<Boolean> colorWidget;
   private SuppliedValueWidget<Boolean> filteredColorWidget;
   private SuppliedValueWidget<Boolean> gameColorWidget;
   private SuppliedValueWidget<Boolean> gameTargetColorWidget;

   private String[] seenColor = { "grey", "blue", "green", "red", "yellow" };

   public ControlPanelSubsystem() {
      m_colorMatcher.addColorMatch(kBlueTarget);
      m_colorMatcher.addColorMatch(kGreenTarget);
      m_colorMatcher.addColorMatch(kRedTarget);
      m_colorMatcher.addColorMatch(kYellowTarget);
      // turnMotor.configFactoryDefault();
      colorWidget = Shuffleboard.getTab("ControlPanel").addBoolean("ColorDetected", () -> true);
      filteredColorWidget = Shuffleboard.getTab("ControlPanel").addBoolean("FilterColor", () -> true);
      gameColorWidget = Shuffleboard.getTab("ControlPanel").addBoolean("GameColor", () -> true);
      gameTargetColorWidget = Shuffleboard.getTab("ControlPanel").addBoolean("OurTargetColor", () -> true);
      // Shuffleboard.getTab("ControlPanel").add("RedValue", redValue);
      //    Shuffleboard.getTab("ControlPanel").add("GreenValue", greenValue);
      //    SmartDashboard.putNumber("BlueValue", blueValue);
      //    Shuffleboard.getTab("ControlPanel").add("RGBSUM", redValue + greenValue + blueValue);

   }

   @Override
   public void periodic() {
      loopCount++;
      if (loopCount > 5) {
         // This method will be called once per scheduler run
         detectedColor = m_colorSensor.getColor();
         double redValue = detectedColor.red;
         double greenValue = detectedColor.green;
         double blueValue = detectedColor.blue;

   
         ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

         if (match.color == kBlueTarget) {
            colorNumber = 1;
         } else if (match.color == kGreenTarget) {
            colorNumber = 2;
         } else if (match.color == kRedTarget) {
            colorNumber = 3;
         } else if (match.color == kYellowTarget) {
            colorNumber = 4;
         } else {
            colorNumber = 0;

         }

         colorWidget.withProperties(Map.of("colorWhenTrue", seenColor[colorNumber]));
         filteredColorWidget.withProperties(Map.of("colorWhenTrue", seenColor[colorNumberFiltered]));
         gameColorWidget.withProperties(Map.of("colorWhenTrue", seenColor[gameColorNumber]));
         int ourTargetColor = gameColorNumber + 2;
         if (ourTargetColor > 4)
            ourTargetColor -= 4;
         gameTargetColorWidget.withProperties(Map.of("colorWhenTrue", seenColor[ourTargetColor]));

         filterColors();
         SmartDashboard.putNumber("CN", colorNumber);

         /**
          * 
          * The sensor returns a raw IR value of the infrared light detected.
          * 
          */

         double IR = m_colorSensor.getIR();
         // Shuffleboard.getTab("ControlPanel").add("IR MM", IR);
         // Shuffleboard.getTab("ControlPanel").add("Confidence", match.confidence);
      }

   }

   public void filterColors() {
      if (colorNumber == 1) {
         blueCount++;
         if (blueCount >= filterNumber) {
            colorNumberFiltered = 1;
            redCount = 0;
            yellowCount = 0;
            greenCount = 0;
         }
      }
      if (colorNumber == 2) {
         greenCount++;
         if (greenCount >= filterNumber) {
            colorNumberFiltered = 2;
            redCount = 0;
            blueCount = 0;
            yellowCount = 0;
         }
      }
      if (colorNumber == 3) {
         redCount++;
         if (redCount >= filterNumber) {
            colorNumberFiltered = 3;
            yellowCount = 0;
            blueCount = 0;
            greenCount = 0;
         }
      }

      if (colorNumber == 4) {
         yellowCount++;

         if (yellowCount >= filterNumber) {
            colorNumberFiltered = 4;
            redCount = 0;
            blueCount = 0;
            greenCount = 0;
         }
      }

   }

   public int getGameData() {
      String gameData;

      gameData = DriverStation.getInstance().getGameSpecificMessage();
      if (gameData.length() > 0) {

         switch (gameData.charAt(0)) {
         case 'B':
            // Blue case code
            gameColorNumber = 1;

            break;
         case 'G':
            // Green case code
            gameColorNumber = 2;
            break;
         case 'R':
            // Red case code
            gameColorNumber = 3;
            break;
         case 'Y':
            // Yellow case code
            gameColorNumber = 4;
            break;
         default:
            // This is corrupt data
            gameColorNumber = 0;
            break;
         }
         SmartDashboard.putNumber("GameColorNumber", gameColorNumber);
      } else {
         // Code for no data received yet

      }
      return gameColorNumber;
   }

   public void turnWheelMotor(double speed) {
      // turnMotor.set(ControlMode.PercentOutput, speed);

   }

}
