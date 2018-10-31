package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Stephen Ogden on 9/20/18.
 * FTC 6128 | 7935
 * FRC 1595
 */

@TeleOp(name = "6128 TeleOp", group = "Official")
public class teleop extends LinearOpMode {

    // Declare the config file, that way we can use the pre-made fictions for cleaner code
    private config robot = new config(this.telemetry);

    public void runOpMode() {

        // Initialize the robot
        robot.ConfigureRobot(this.hardwareMap);

        // Wait for the start button to be pressed
        waitForStart();

        // Code that we want to run repeatedly
        while (opModeIsActive()) {

            // Copy paste from Henry's Iterative program, with some formatting changes :)

            // Setup a variable for each drive wheel to save power level for telemetry
            double left1Power, right1Power, left2Power, right2Power, allPower = 1;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            double driveForward = gamepad1.left_stick_y, driveRightSide = gamepad1.left_stick_x, turnRight = -gamepad1.right_stick_x;


            left1Power = Range.clip((-driveRightSide + driveForward + turnRight) * allPower, -1.0, 1.0);
            right1Power = Range.clip((driveRightSide + driveForward - turnRight) * allPower, -1.0, 1.0);
            left2Power = Range.clip((driveRightSide + driveForward + turnRight) * allPower, -1.0, 1.0);
            right2Power = Range.clip((-driveRightSide + driveForward - turnRight) * allPower, -1.0, 1.0);

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.

            // Send calculated power to wheels
            robot.left1.setPower(left1Power);
            robot.right1.setPower(right1Power);
            robot.left2.setPower(left2Power);
            robot.right2.setPower(right2Power);

            if (gamepad1.dpad_up) {
                robot.arm.setPower(1);
            } else if (gamepad1.dpad_down) {
                robot.arm.setPower(-1);
            } else {
                robot.arm.setPower(0);
            }

            // Update telemetry
            robot.updateTelemetry();
        }

    }
}
