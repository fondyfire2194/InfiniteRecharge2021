/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// to see what's connected to the CAN Bus and what's not
public class CANChecker {
    private final ArrayList<BaseTalon> talons;

    private final ArrayList<Integer> sparkMaxID;

    private final ArrayList<CANSparkMax> sparkMax;

    private final String NAME = "CAN Bus";

    private int i = -1; // 1 will get added on first iteration
    private final int UPDATE_PERIOD = 25; // every 25 iterations, called at 50Hz = twice a second

    public CANChecker() {
        talons = new ArrayList<BaseTalon>();
        sparkMax = new ArrayList<CANSparkMax>();
        sparkMaxID = new ArrayList<Integer>();

    }

    public void addTalons(List<BaseTalon> newTalons) {
        talons.addAll(newTalons);
    }

    public void addSparkMax(List<CANSparkMax> newSparkMax) {
        sparkMax.addAll(newSparkMax);
    }

    public void addSparkMaxID(List<Integer> newSparkMaxID) {
        sparkMaxID.addAll(newSparkMaxID);
    }

    public void periodic() {
        i++;
        i = i % UPDATE_PERIOD;
        if (i == 0) {
            checkTalons();
          checkSparkMax();
        }
    }

    public void checkTalons() {
        boolean allGood = true;

        for (int i = 0; i < talons.size(); i++) {
            BaseTalon talon = talons.get(i);
            int id = talon.getDeviceID();

            int param = talon.configGetCustomParam(0);
            boolean connected = (param == id);

            SmartDashboard.putBoolean(NAME + "/" + id, connected);
            if (!connected) {
                // in case it hasn't been configured or maybe it was changed
                talon.configSetCustomParam(id, 0); // set the parameter to its ID
                allGood = false;
            }
        }

        SmartDashboard.putBoolean(NAME + "/All good", allGood); // probably want to do something with it
    }

    public void checkSparkMax() {

        boolean allSparksGood = true;

        for (int i = 0; i < sparkMaxID.size(); i++) {

            int id = sparkMaxID.get(i);

            CANSparkMax motor = sparkMax.get(i);

            boolean sparkConnected = motor.getDeviceId() == id;

            if (!sparkConnected)
                SmartDashboard.putString("SparkMax " + String.valueOf(id), " not Connected");

            SmartDashboard.putBoolean("AllSparksGood", allSparksGood);

        }

    }

}
