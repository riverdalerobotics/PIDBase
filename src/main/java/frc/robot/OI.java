/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Add your docs here.
 */
public class OI {
    XboxController drive = new XboxController(0);

    public double getMove(){
        return -drive.getY(Hand.kLeft);
    }
    public double getTurn(){
        return drive.getX(Hand.kRight);
    }

    public boolean isReset(){
        return drive.getStartButton();
    }
    public boolean isGoDistance(){
        return drive.getYButton();
    }
    public boolean goPid(){
        return drive.getXButton();
    }
    public boolean endPid(){
        return drive.getBButton();
    }
    public boolean turn(){
        return drive.getBumper(Hand.kRight);
    }
}
