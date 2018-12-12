package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Stephen Ogden on 11/30/18.
 * FTC 6128 | 7935
 * FRC 1595
 */

@Deprecated
@Disabled
@TeleOp(name = "6128 TeleOp (Only 1 controller)", group = "Official")
public class TeleOpSingleController extends LinearOpMode {

    // Declare the RobotConfig file, that way we can use the pre-made fictions for cleaner code
    private RobotConfig robot = new RobotConfig(this.telemetry);

    public void runOpMode() {

        // Initialize the robot
        robot.configureRobot(this.hardwareMap);

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {

            // Calculate the power value and then send power to motors
            robot.left1.setPower(Range.clip(((-1 * gamepad1.left_stick_x) + gamepad1.left_stick_y + (-1 * gamepad1.right_stick_x)), -1.0, 1.0));
            robot.right1.setPower(Range.clip((gamepad1.left_stick_x + gamepad1.left_stick_y - (-1 * gamepad1.right_stick_x)), -1.0, 1.0));
            robot.left2.setPower(Range.clip((gamepad1.left_stick_x + gamepad1.left_stick_y + (-1 * gamepad1.right_stick_x)), -1.0, 1.0));
            robot.right2.setPower(Range.clip(((-1 * gamepad1.left_stick_x) + gamepad1.left_stick_y - (-1 * gamepad1.right_stick_x)), -1.0, 1.0));


            // Climb to predefined positions
            if (gamepad1.y) {
                robot.climber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.climber.setTargetPosition(robot.maxClimberPos);
            } else if (gamepad1.x) {
                robot.climber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.climber.setTargetPosition(robot.minClimberPos);
            }

            if (robot.climber.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)) {
                if (robot.isThere(1, robot.climber)) {
                    robot.climber.setPower(0);
                } else {
                    robot.climber.setPower(1);
                }
            }


            if (gamepad1.dpad_up || gamepad1.dpad_down) {
                robot.climber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }


            if (robot.climber.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)) {
                if (gamepad1.dpad_down) {
                    robot.climber.setPower(-1);
                } else if (gamepad1.dpad_up) {
                    robot.climber.setPower(1);
                } else {
                    robot.climber.setPower(0);
                }
            }

            robot.arm.setPower(Range.clip(gamepad2.left_stick_y, -0.6, 0.6));

            // Set the intake to the power left trigger compared to the right trigger
            robot.intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);


            // Update telemetry
            robot.updateTelemetry();
        }

        // Stop tracking gold
        robot.goldDetector.disable();

    }
}
