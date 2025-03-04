package com.team1678.frc2024.subsystems;

import org.littletonrobotics.junction.AutoLog;

import com.team1678.frc2024.controlboard.ControlBoard;
import com.team6647.subsystems.Superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class PeriodicLogs {
    @AutoLog
    public static class Inputs{
        public static boolean mFeederBreak = false;
    }
    static ControlBoard mControlBoard = ControlBoard.getInstance();

    public static Superstructure superstructure = Superstructure.getInstance();
    public static void periodic(){
        SmartDashboard.putBoolean("plogs/rightBumper", mControlBoard.driver.rightBumper.isBeingPressed());
    }
}
