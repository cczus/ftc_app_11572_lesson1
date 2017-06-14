/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
    Team:       10355 - Project Peacock
    Autonomous Program - Red Strategy #1
    Alliance Color: Red
    Robot Starting Position: Red zone, wall near ramp
    Strategy Description:
        - Press correct button on beacon near ramp
        - Press correct button on beacon furthest from ramp
        - Park a wheel on the red ramp

    Hardware Setup:
        - 4 mecanum wheels with encoder on LF wheel - encoder utilized for measuring distance for fwd/rev drive operation
        - Arm Motor with encoder - controls mechanism for dumping particles into ramp
        - Gyro sensor located at the center of the robot - utilized to compensate for drift
        - 1 x Color sensor (colorSensorLeft)- utilized to identify beacon color
        - 1 x Touch sensor - utilized to identify when robot touches wall with the front of the robot
        - 1 x Optical Distance Sensor (ODS) - utilized to locate the white lines on the floor
        - 1 x Motorola Camera - Utilized for Vuforia positioning of the robot on the field

    State Order:
        - ACQUIRE_BLUE_BEACON_RIGHT       // moves from the wall to the first beacon closest to the ramp
        - PUSH_BUTTON_RED               // Identifies which button to press, right or left
        - PRESS_BUTTON                  // presses the correct button - also validates that the button is pressed
                                        // and attempts to press over again until the button is pressed
        - ACQUIRE_BLUE_BEACON_LEFT      // moves from the wall to the second beacon on the right of the field
        - PUSH_BUTTON_RED               // Identifies which button to press, right or left
        - PRESS_BUTTON                  // presses the correct button - also validates that the button is pressed
                                        // and attempts to press over again until the button is pressed
        - END_GAME                      // identifies the last actions before the end of autonomous mode
        - RAMP                          // For this strategy, the robot will end with a wheel parked on the ramp
        - HALT                          // Shutdown sequence for autonomous mode

 */
package org.firstinspires.ftc.teamcode.Opmodes;

/**
 * Import the classes we need to have local access to.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.HardwareProfiles.HardwareTestPlatform;
import org.firstinspires.ftc.teamcode.Libs.Beacon;
import org.firstinspires.ftc.teamcode.Libs.DataLogger;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;
import org.firstinspires.ftc.teamcode.Libs.Shooter;
import org.firstinspires.ftc.teamcode.Libs.VuforiaLib;

import java.util.List;

/**
 * Name the opMode and put it in the appropriate group
 */
@Autonomous(name = "Mecanum Template", group = "COMP")

public class MecanumTemplate extends LinearOpMode {

    /**
     * Instantiate all objects needed in this class
     */

    private final static HardwareTestPlatform robot = new HardwareTestPlatform();
    double radians = 0;
    private VuforiaLib myVuforia = new VuforiaLib();
    private ElapsedTime runtime = new ElapsedTime();
    private LinearOpMode opMode = this;
    Beacon beacon = new Beacon(robot, opMode);
    Shooter shooter = new Shooter(robot, opMode);

    /**
     * Define global variables
     */
    private double mm = 500;            //Distance for translateDistance method
    private double power = .6;          //Motor power for all methods
    private double heading = 90;        //Heading for all methods
    private double y = -200;            //Vuforia y stop coordinate
    private double x = -200;            //Vuforia x stop coordinate
    private double changeSpeed = 0;     //Rotation speed for motor speed calculations
    private double initZ;               //The initial reading from integrated Z
    private double currentZint;         //Current integrated Z value
    private double zCorrection;         //Course correction in degrees
    private double timeOut = 5;         //Timeout in seconds for translateTime method
    private double timeOutTime;         //Calculated time to stop
    private String procedure;           //Name of the method that is running
    private double odsThreshold = .3;   //Threshold at which the ODS sensor acquires the whie line
    private double ods = 0;             //Value returned from the Optical Distance Sensor
    private double colorRightRed = 0;   //Value from color sensor
    private double colorRightBlue = 0;  //Value from color sensor
    private double colorLeftRed = 0;    //Value from color sensor
    private double colorLeftBlue = 0;   //Value from color sensor
    private double robotX;              // The robot's X position from VuforiaLib
    private double robotY;              // The robot's Y position from VuforiaLib
    private double robotBearing;        //Bearing to, i.e. the bearing you need to steer toward
    private double LF, RF, LR, RR;      //Motor power setting
    private double myCurrentMotorPosition;  //Current encoder position
    private double myTargetPosition;        //Target encoder position
    private boolean colorLedEnable = false; //Enable if in testing mode, disable for beacon
    private boolean leftAlign = false;      //
    private boolean beaconFlag = false;
    private List<Double> vuforiaTracking;   //List of Vuforia coordinates
    private List<VuforiaTrackable> myTrackables;    //List of Vuforia trackable objects
    private DataLogger Dl;                          //Datalogger object
    private double motorCorrectCoefficient = .05;    //Amount to divide the zInt drift by
    private double motorCorrectCoefficient2 = .025;    //Amount to divide the zInt drift by
    private String beaconColorRight;                //Color of the right side of the beacon
    private String beaconColorLeft;                 //Color of the left side of the beacon
    private String button;                          //Which button to push
    private String alliance = "blue";                //Your current alliance
    private boolean beaconState = false;            //Has the beacon been triggered?
    private String courseCorrect = "";
    private State state = State.HALT;    //Machine State
    private String nextState = "Beacon2";
    private int target;


    public void runOpMode() {
        /**
         * Setup the init state of the robot.  This configures all the hardware that is defined in
         * the HardwareTestPlatform class.
         */
        robot.init(hardwareMap);
        createDl();

        /**
         * Initialize Vuforia and retrieve the list of trackable objects.
         */
        telemetry.addData("Waiting on Vuforia", "");
        telemetry.update();

        myTrackables = myVuforia.vuforiaInit();

        DriveMecanum drive = new DriveMecanum(robot, opMode, myVuforia, myTrackables, Dl);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(1000);

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        /**
         * Start the opMode
         */
        waitForStart();

        while (opModeIsActive()) {
            /**
             * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
             * This is the section of code you should change for your robot.
             * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
             */
            // FOO
            switch (state) {
                case DRIVE:
                    /**
                    *Calls the drive.translateDistance method you must pass in the following variables
                     * mm as int
                     * power as double 0 to 1
                     * heading as int 0 to 359
                    **/
                    drive.translateDistance(100, 1, 0);

                    /**
                     * Pause for 5000 milliseconds
                     */
                    sleep(5000);

                    /**
                     * Set the state to HALT
                     */
                    state = state.HALT;
                    break;
                case HALT:
                    /**
                     * Stop all drive motors
                     */
                    drive.motorsHalt();               //Stop the motors

                    /**
                     * Stop the data logger
                     **/
                    dlStop();

                    /**
                     * Exit the opmode
                     */
                    requestOpModeStop();
                    break;
            }
        }
    }

    /**
     * Setup the dataLogger
     * The dataLogger takes a set of fields defined here and sets up the file on the Android device
     * to save them to.  We then log data later throughout the class.
     */
    private void createDl() {

        Dl = new DataLogger("AutoMecanumSimpleTest" + runtime.time());
        Dl.addField("runTime");
        Dl.addField("Alliance");
        Dl.addField("State");
        Dl.addField("Procedure");
        Dl.addField("courseCorrect");
        Dl.addField("heading");
        Dl.addField("robotX");
        Dl.addField("robotY");
        Dl.addField("X");
        Dl.addField("Y");
        Dl.addField("robotBearing");
        Dl.addField("initZ");
        Dl.addField("currentZ");
        Dl.addField("zCorrection");
        Dl.addField("touchSensor");
        Dl.addField("ODS");
        Dl.addField("colorRightRed");
        Dl.addField("colorRightBlue");
        Dl.addField("colorLeftRed");
        Dl.addField("colorLeftBlue");
        Dl.addField("LFTargetPos");
        Dl.addField("LFMotorPos");
        Dl.addField("LF");
        Dl.addField("RF");
        Dl.addField("LR");
        Dl.addField("RR");
        Dl.newLine();
    }

    /**
     * Log data to the file on the phone.
     */
    private void logData() {

        Dl.addField(String.valueOf(runtime.time()));
        Dl.addField(String.valueOf(alliance));
        Dl.addField(String.valueOf(state));
        Dl.addField(String.valueOf(procedure));
        Dl.addField(String.valueOf(courseCorrect));
        Dl.addField(String.valueOf(heading));
        Dl.addField(String.valueOf((int) robotX));
        Dl.addField(String.valueOf((int) robotY));
        Dl.addField(String.valueOf(x));
        Dl.addField(String.valueOf(y));
        Dl.addField(String.valueOf((int) robotBearing));
        Dl.addField(String.valueOf(initZ));
        Dl.addField(String.valueOf(currentZint));
        Dl.addField(String.valueOf(zCorrection));
        Dl.addField(String.valueOf(robot.touchSensor.getValue()));
        Dl.addField(String.valueOf(ods));
        Dl.addField(String.valueOf(colorRightRed));
        Dl.addField(String.valueOf(colorRightBlue));
        Dl.addField(String.valueOf(colorLeftRed));
        Dl.addField(String.valueOf(colorLeftBlue));
        Dl.addField(String.valueOf(myTargetPosition));
        Dl.addField(String.valueOf(robot.motorLF.getCurrentPosition()));
        Dl.addField(String.valueOf(LF));
        Dl.addField(String.valueOf(RF));
        Dl.addField(String.valueOf(LR));
        Dl.addField(String.valueOf(RR));
        Dl.newLine();
    }

    /**
     * Stop the DataLogger
     */
    private void dlStop() {
        Dl.closeDataLogger();

    }

    /**
     * Enumerate the States of the machine.
     */
    enum State {
        HALT, DRIVE
    }

}
