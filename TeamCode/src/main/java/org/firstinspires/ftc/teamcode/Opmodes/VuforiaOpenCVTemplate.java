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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
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
@Autonomous(name = "Vuforia Open CV Template", group = "COMP")
@Disabled
public class VuforiaOpenCVTemplate extends LinearOpMode {

    /**
     * Instantiate all objects needed in this class
     */

    private VuforiaLib myVuforia = new VuforiaLib();
    private Vuforia vuforia = new Vuforia();
    private ElapsedTime runtime = new ElapsedTime();
    private LinearOpMode opMode = this;

    /**
     * Define global variables
     */
    private List<Double> vuforiaTracking;   //List of Vuforia coordinates
    private List<VuforiaTrackable> myTrackables;    //List of Vuforia trackable objects
    private DataLogger Dl;                          //Datalogger object
    private State state = State.HALT;    //Machine State

    public void runOpMode() {
        createDl();

        /**
         * Initialize Vuforia and retrieve the list of trackable objects.
         */
        telemetry.addData("Waiting on Vuforia", "");
        telemetry.update();

        myTrackables = myVuforia.vuforiaInit();

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
                case OPENCV:

                    break;
                case HALT:

                    //Stop the DataLogger
                    dlStop();

                    //Exit the OpMode
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
        Dl.newLine();
    }

    /**
     * Log data to the file on the phone.
     */
    private void logData() {

        Dl.addField(String.valueOf(runtime.time()));
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
        HALT, OPENCV
    }

}
