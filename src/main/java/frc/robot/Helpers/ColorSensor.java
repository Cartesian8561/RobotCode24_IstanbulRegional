// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Helpers;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class ColorSensor {
    private ColorSensorV3 colorSensor;
    private Color note;
    private ColorMatch match;

    public ColorSensor() {
        try{
            colorSensor  = new ColorSensorV3(Port.kOnboard);
            note = new Color(129, 98, 27);
            match = new ColorMatch();
            match.addColorMatch(note);
        }catch(Exception e){}
    }

    public boolean matchColor() {
        Color detectedColor = colorSensor.getColor();
        if (colorSensor != null){
            return (Math.abs(detectedColor.red - note.red) + Math.abs(detectedColor.green - note.green) + Math.abs(detectedColor.blue - note.blue) < 0.4);
        }else{
        return false;
        }
    }

    public Color getDetectedColour() {
        return colorSensor.getColor();
    }

}
