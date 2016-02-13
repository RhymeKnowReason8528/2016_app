package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by RobotK on 2/13/2016.
 */
public class ComparisonTest {
    public static void main(String[] args) {
        RKRGyro.Comparison comparison = RKRGyro.Comparison.LESS_THAN;

        if(comparison.evaluate(0, 90)) {
            System.out.println("Less than comparison passed");
        }
        else {
            System.out.println("Less than comparison failed");
        }
    }
}
