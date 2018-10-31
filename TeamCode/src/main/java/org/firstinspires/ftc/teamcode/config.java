package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * This is the config file for the robot. It also has (or will have) many useful functions in it.
 * <p>
 * For example rather than typing: <br>
 * <br>
 * <code>left_front = hardware.dcMotor.get("left_drive");<br>
 * right_front = hardware.dcMotor.get("right_drive");<br>
 * ...</code>
 * <br>
 * <br>
 * you can just type: <br>
 * <br>
 * <code>private config robot = new config();<br>
 * robot.ConfigureRobotHardware(this.hardwareMap);</code>
 * <br>
 * <br>
 * That's literally the configuration process done!
 * <br>
 * <br>
 * <br>
 * <p>
 * <p>
 * Created by Stephen Ogden on 9/13/18.
 * <p>
 * FTC 6128 | 7935
 * FRC 1595
 */
class config {

    DcMotor left1, right1, left2, right2;

    private Telemetry telemetry;

    config(Telemetry t) {
        this.telemetry = t;
    }

    void ConfigureRobot(HardwareMap hardware) {

        // Declare and setup left1
        status("Setting up left1");
        left1 = hardware.dcMotor.get("left1");
        left1.setZeroPowerBehavior(BRAKE);
        left1.setMode(RUN_USING_ENCODER);
        left1.setDirection(REVERSE);

        // Declare and setup right1
        status("Setting up right1");
        right1 = hardware.dcMotor.get("right1");
        right1.setZeroPowerBehavior(BRAKE);
        right1.setMode(RUN_USING_ENCODER);
        right1.setDirection(FORWARD);

        // Declare and setup left2
        status("Setting up left2");
        left2 = hardware.dcMotor.get("left2");
        left2.setZeroPowerBehavior(BRAKE);
        left2.setMode(RUN_USING_ENCODER);
        left2.setDirection(REVERSE);

        // Declare and setup right2
        status("Setting up right2");
        right2 = hardware.dcMotor.get("right2");
        right2.setZeroPowerBehavior(BRAKE);
        right2.setMode(RUN_USING_ENCODER);
        right2.setDirection(FORWARD);

        // Update telemetry to signal done!
        status("Ready!");

    }

    void updateTelemetry() {

        if (left1 != null) {
            telemetry.addData("Left1 (target)", String.format(Locale.US, "%.4d (%.4d)", left1.getCurrentPosition(), left1.getTargetPosition()));
        }

        if (right1 != null) {
            telemetry.addData("Right1 (target)", String.format(Locale.US, "%.4d (%.4d)", right1.getCurrentPosition(), right1.getTargetPosition()));
        }

        if (left2 != null) {
            telemetry.addData("Left2 (target)", left2.getCurrentPosition());
        }

        if (right2 != null) {
            telemetry.addData("Right2 (target)", right2.getCurrentPosition());
        }

        telemetry.update();
    }

    private void status(String string) {
        telemetry.addData("Status", string);
        telemetry.update();
    }

}