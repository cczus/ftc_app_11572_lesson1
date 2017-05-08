package org.firstinspires.ftc.teamcode.Opmodes11572;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareProfiles.HardwareTestPlatform;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 * <p>
 * This code assumes a Servo configured with the name "left claw" as is found on a pushbot.
 * <p>
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "SETUP - Check Motors", group = "SETUP")
@Disabled
public class AutoSetupMotors extends LinearOpMode {
    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    private final static HardwareTestPlatform robot = new HardwareTestPlatform();
    private static final int CYCLE_MS = 50;     // period of each cycle
    private static final double MAX_POS = 1.0;     // Maximum rotational position
    private static final double MIN_POS = 0.0;     // Minimum rotational position
    private static final double LEFT_MAX = .6;
    private static final double RIGHT_MAX = .4;
    private ElapsedTime runtime = new ElapsedTime();
    private double interval = 2;
    private double stopTime = 0;

    // Define class members
    private Servo servo;
    private double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.servoODS.setPosition(0);
        robot.servoPusher.setPosition(.5);
        // Wait for the start button
        telemetry.addData(">", "Press Start to test Motors.");
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {
            stopTime = (runtime.time() + interval);
            while (runtime.time() < stopTime) {
                robot.motorLF.setPower(1);
                idle();
            }
            motorsHalt();
            sleep(2000);

            stopTime = (runtime.time() + interval);
            while (runtime.time() < stopTime) {
                robot.motorLR.setPower(1);
                idle();
            }
            motorsHalt();
            sleep(2000);

            stopTime = (runtime.time() + interval);
            while (runtime.time() < stopTime) {
                robot.motorRF.setPower(1);
                idle();
            }
            motorsHalt();
            sleep(2000);

            stopTime = (runtime.time() + interval);
            while (runtime.time() < stopTime) {
                robot.motorRR.setPower(1);
                idle();
            }
            motorsHalt();
            sleep(2000);

            stopTime = (runtime.time() + interval);
            while (runtime.time() < stopTime) {
                robot.motorFeeder.setPower(1);
                idle();
            }
            motorsHalt();
            sleep(2000);

            stopTime = (runtime.time() + interval);
            while (runtime.time() < stopTime) {
                robot.motorShooter.setPower(1);
                idle();
            }
            motorsHalt();
            sleep(2000);

            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();

    }

    public void motorsHalt() {
        robot.motorLF.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRR.setPower(0);
        robot.motorFeeder.setPower(0);
    }
}
