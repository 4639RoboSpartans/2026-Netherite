/* Copyright (c) 2025-2026 FRC 4639. */

package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

public class LoadShooterLookupTableData {
    public static void loadData(InterpolatingDoubleTreeMap distToRPM, InterpolatingDoubleTreeMap distToAngle, InterpolatingDoubleTreeMap distToTOF) {
        String line = "";
        String splitBy = ",";
        try (BufferedReader br = new BufferedReader(new FileReader("src/test/resources/frc/robot/constants/shooterTestData.csv"))) {
            while ((line = br.readLine()) != null) { // returns a Boolean value
                String[] data = line.split(splitBy); // use comma as separator
                double dist = Integer.parseInt(data[0]);
                double angle = Double.parseDouble(data[1]);
                double rpm = Double.parseDouble(data[2]);
                double tof = Double.parseDouble(data[3]);
                distToRPM.put(dist, rpm);
                distToAngle.put(dist, angle);
                distToTOF.put(dist, tof);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
