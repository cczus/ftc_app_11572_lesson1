package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareProfiles.HardwareTestPlatform;

/**
 * Created by caseyzandbergen on 12/1/16.
 */

public class Beacon {
    private HardwareTestPlatform robot = null;
    private LinearOpMode opMode = null;
    private double red = 0;
    private double blue = 0;
    private String beaconColor = "unk";

    public Beacon(HardwareTestPlatform myRobot, LinearOpMode myOpMode) {
        robot = myRobot;
        opMode = myOpMode;
    }

    public String getRightColor(boolean colorLedEnable) {
        String color = "unk";
        double red;
        double blue;
        String beaconColorRight = "unk";

        if (colorLedEnable) {
            robot.colorSensorRight.enableLed(true);
        } else {
            robot.colorSensorRight.enableLed(false);
        }

        opMode.sleep(100);

        while (color.equals("unk")) {
            red = robot.colorSensorRight.red();
            blue = robot.colorSensorRight.blue();

            if (red >= 2) {
                color = "red";
            }
            if (blue >= 2) {
                color = "blue";
            }
            beaconColorRight = color;

            opMode.idle();


        }
        return beaconColorRight;
    }


    public String getButtonPush(String alliance, String beaconColorRight) {
        String button = "unk";
        if (beaconColorRight.equals("red") && alliance.equals("red")) {
            button = "right";
        }

        if (beaconColorRight.equals("blue") && alliance.equals("red")) {
            button = "left";
        }

        if (beaconColorRight.equals("red") && alliance.equals("blue")) {
            button = "left";
        }

        if (beaconColorRight.equals("blue") && alliance.equals("blue")) {
            button = "right";
        }

        return button;
    }

    public void pushButton(String button) {
        if (button.equals("right")) {
            robot.servoPusher.setPosition(0);
        }

        if (button.equals("left")) {
            robot.servoPusher.setPosition(1);
        }
    }

    public boolean checkBeacon(boolean colorLedEnable, String alliance) {
        boolean beaconState = false;
        double left = 0;
        double right = 0;

        if (colorLedEnable) {
            robot.colorSensorRight.enableLed(true);
            robot.colorSensorLeft.enableLed(true);
        } else {
            robot.colorSensorRight.enableLed(false);
            robot.colorSensorLeft.enableLed(false);
        }

        if (alliance.equals("red")) {
            if (robot.colorSensorRight.red() >= 2 && robot.colorSensorLeft.red() >= 2) {  //Beacon activated
                beaconState = true;
            }
            opMode.telemetry.addData("colorSensorRight", String.valueOf(robot.colorSensorRight.red()));
            opMode.telemetry.addData("colorSensorLeft", String.valueOf(robot.colorSensorLeft.red()));
            opMode.telemetry.addData("beaconState", String.valueOf(beaconState));
        }

        if (alliance.equals("blue")) {
            if (robot.colorSensorRight.blue() >= 2 && robot.colorSensorLeft.blue() >= 2) {  //Beacon activated
                beaconState = true;


            }
            opMode.telemetry.addData("colorSensorRight", String.valueOf(robot.colorSensorRight.blue()));
            opMode.telemetry.addData("colorSensorLeft", String.valueOf(robot.colorSensorLeft.blue()));
            opMode.telemetry.addData("beaconState", String.valueOf(beaconState));
        }
        return beaconState;
    }

    public void resetButton() {
        robot.servoPusher.setPosition(.5);
    }

}
