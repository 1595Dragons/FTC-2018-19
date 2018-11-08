package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Stephen Ogden on 11/1/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
@TeleOp(name = "Drive distance funcion test", group = "Test")
public class driveDistanceTest extends LinearOpMode {

    private RobotConfig robot = new RobotConfig(this.telemetry);

    @Override
    public void runOpMode() {

        robot.configureRobot(this.hardwareMap);
        robot.resetMotors(robot.left1, robot.right2, robot.left2, robot.right1);

        int error = 5, distance = 24;

        boolean manual = true;

        MecanumDriveDirection direction = MecanumDriveDirection.FORWARD;

        waitForStart();
        while (opModeIsActive() && !gamepad1.b && !gamepad1.y) {

            if (manual) {
                // Setup a variable for each drive wheel to save power level for telemetry
                double left1Power, right1Power, left2Power, right2Power, allPower = 1;

                double driveForward = gamepad1.left_stick_y, driveRightSide = gamepad1.left_stick_x, turnRight = gamepad1.right_stick_x;


                // Calculate the power to send to each motor
                left1Power = Range.clip((-driveRightSide + driveForward + turnRight) * allPower, -1.0, 1.0);
                right1Power = Range.clip((driveRightSide + driveForward - turnRight) * allPower, -1.0, 1.0);
                left2Power = Range.clip((driveRightSide + driveForward + turnRight) * allPower, -1.0, 1.0);
                right2Power = Range.clip((-driveRightSide + driveForward - turnRight) * allPower, -1.0, 1.0);


                // Send calculated power to wheels
                robot.left1.setPower(left1Power);
                robot.right1.setPower(right1Power);
                robot.left2.setPower(left2Power);
                robot.right2.setPower(right2Power);

            } else {
                if (gamepad1.a || gamepad1.x) {
                    robot.driveDistance(direction, distance, .75);
                }

                if (robot.isThere(error, robot.left1, robot.left2, robot.right1, robot.right2)) {
                    robot.resetMotors(robot.left1, robot.left2, robot.right1, robot.right2);
                }
            }

            if (gamepad1.dpad_right || gamepad1.dpad_left) {
                manual = !manual;
                if (manual) {
                    robot.setMaxPower(0, robot.left1, robot.right1, robot.left2, robot.right2);
                    robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                } else {
                    robot.resetMotors(robot.left1, robot.right1, robot.left2, robot.right2);
                }
            }

            if (gamepad1.dpad_up) {
                if (distance < 60) {
                    distance++;
                }
            } else if (gamepad1.dpad_down) {
                if (distance > 0) {
                    distance--;
                }
            }

            if (gamepad1.left_bumper) {
                if (direction.ordinal() + 1 >= MecanumDriveDirection.values().length) {
                    direction = MecanumDriveDirection.values()[0];
                } else {
                    direction = MecanumDriveDirection.values()[direction.ordinal() + 1];
                }
            } else if (gamepad1.right_bumper) {
                if (direction.ordinal() - 1 < 0) {
                    direction = MecanumDriveDirection.values()[MecanumDriveDirection.values().length - 1];
                } else {
                    direction = MecanumDriveDirection.values()[direction.ordinal() - 1];
                }
            }

            telemetry.addData("Manual", Boolean.toString(manual))
                    .addData("Direction", direction.name())
                    .addData("Distance", distance);
            robot.updateTelemetry();

        }
    }
}
