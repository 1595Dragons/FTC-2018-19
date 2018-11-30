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
public class Teleop extends LinearOpMode {

    // Declare the RobotConfig file, that way we can use the pre-made fictions for cleaner code
    private RobotConfig robot = new RobotConfig(this.telemetry);

    public void runOpMode() {

        // Initialize the robot
        robot.configureRobot(this.hardwareMap);

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {

            // Setup a variable for each drive wheel
            double left1Power,
                    right1Power,
                    left2Power,
                    right2Power,
                    driveForward = gamepad1.left_stick_y,
                    driveRightSide = gamepad1.left_stick_x,
                    turnRight = gamepad1.right_stick_x;


            // Calculate the power to send to each motor
            left1Power = Range.clip((-driveRightSide + driveForward + turnRight), -1.0, 1.0);
            right1Power = Range.clip((driveRightSide + driveForward - turnRight), -1.0, 1.0);
            left2Power = Range.clip((driveRightSide + driveForward + turnRight), -1.0, 1.0);
            right2Power = Range.clip((-driveRightSide + driveForward - turnRight), -1.0, 1.0);


            // Send calculated power to wheels
            robot.left1.setPower(left1Power);
            robot.right1.setPower(right1Power);
            robot.left2.setPower(left2Power);
            robot.right2.setPower(right2Power);


            // Climb to predefined positions
            if (gamepad2.y) {
                robot.climber.setTargetPosition(robot.maxClimberPos);
            } else if (gamepad2.x) {
                robot.climber.setTargetPosition(robot.minClimberPos);
            }
            if (robot.isThere(1, robot.climber)) {
                robot.climber.setPower(0);
            } else {
                robot.climber.setPower(1);
            }

            // Set the intake to the power left trigger compared to the right trigger
            robot.intake.setPower(gamepad2.left_trigger - gamepad2.right_trigger);


            // Set the arm power to that of the left stick, but cap it at 60%
            if (Math.abs(robot.climber.getCurrentPosition() - robot.maxClimberPos) < 100) {
                robot.arm.setPower(Range.clip(gamepad2.left_stick_y, -0.6, 0.6));
            } else {
                robot.arm.setPower(0);
            }


            // Update telemetry
            robot.updateTelemetry();

        }

        // Stop tracking gold
        robot.goldDetector.disable();

    }
}
