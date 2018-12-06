package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Stephen Ogden on 9/20/18.
 * edited by Henry Xu.
 * FTC 7935
 * FRC 1595
 */


@TeleOp(name = "7935 TeleOp", group = "Official")
public class Teleop extends LinearOpMode {

    private Config robot = new Config(this);

    public void runOpMode() {

        // Initialize the robot
        robot.ConfigureRobtHardware(false);


        //Servo positions
        double LeftServoClose = 0.2, LeftServoHalfOpen = 0.6, LeftServoOpen = 0.80;
        double RightServoClose = 0.7, RightServoHalfOpen = 0.35, RightServoOpen = 0.15;


        //MOTORS Power
        double speedForTurn = 0.4, speedForMove = 0.5, speedForSide = 0.7, intakePower = 1, armPower = 1, extendPower = 0.8;


        double left1Power, right1Power, left2Power, right2Power, allPower, armExtend, armUp;

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {

            double driveForward = gamepad2.left_stick_y * speedForMove, driveRightSide = gamepad2.left_stick_x * speedForSide,
                    turnRight = -gamepad2.right_stick_x * speedForTurn;

            //prevent small input from stick
            driveForward = (driveForward >= -0.1 && driveForward <= 0.1) ? 0 : driveForward;
            driveRightSide = (driveRightSide >= -0.1 && driveRightSide <= 0.1) ? 0 : driveRightSide;
            turnRight = (turnRight >= -0.1 && turnRight <= 0.1) ? 0 : turnRight;

            // Set the power to half if the bumpers are pressed
            allPower = (gamepad2.left_bumper || gamepad2.right_bumper) ? 1 : 0.4;


            // Drive with either the D-pad or the joy-sticks
            if (gamepad2.dpad_up) {
                left1Power = -1 * speedForMove * allPower;
                left2Power = -1 * speedForMove * allPower;
                right1Power = -1 * speedForMove * allPower;
                right2Power = -1 * speedForMove * allPower;
            } else if (gamepad2.dpad_down) {
                left1Power = 1 * speedForMove * allPower;
                left2Power = 1 * speedForMove * allPower;
                right1Power = 1 * speedForMove * allPower;
                right2Power = 1 * speedForMove * allPower;
            } else if (gamepad2.dpad_right) {
                left1Power = -1 * speedForSide * allPower;
                left2Power = 1 * speedForSide * allPower;
                right1Power = 1 * speedForSide * allPower;
                right2Power = -1 * speedForSide * allPower;
            } else if (gamepad2.dpad_left) {
                left1Power = 1 * speedForSide * allPower;
                left2Power = -1 * speedForSide * allPower;
                right1Power = -1 * speedForSide * allPower;
                right2Power = 1 * speedForSide * allPower;
            } else {
                left1Power = Range.clip((-driveRightSide + driveForward + turnRight) * allPower, -1.0, 1.0);
                right1Power = Range.clip((driveRightSide + driveForward - turnRight) * allPower, -1.0, 1.0);
                left2Power = Range.clip((driveRightSide + driveForward + turnRight) * allPower, -1.0, 1.0);
                right2Power = Range.clip((-driveRightSide + driveForward - turnRight) * allPower, -1.0, 1.0);
            }

            armUp = (gamepad1.left_stick_y) * armPower;


            armExtend = (gamepad1.dpad_up) ? extendPower : (gamepad1.dpad_down ? -extendPower : 0);


            // Send calculated power to wheels
            robot.left_front.setPower(left1Power);
            robot.right_front.setPower(right1Power);
            robot.left_back.setPower(left2Power);
            robot.right_back.setPower(right2Power);


            robot.armMotorL.setPower(armUp);
            robot.armMotorR.setPower(armUp);


            robot.armMotorExtend.setPower(armExtend);


            if (gamepad1.right_bumper) {
                robot.IO_Motor.setPower(intakePower);
                robot.IO_Servo_Left.setPosition(LeftServoClose);
                robot.IO_Servo_Right.setPosition(RightServoClose);
            } else {
                if (gamepad1.left_bumper) {
                    robot.IO_Motor.setPower(-intakePower);
                } else {
                    robot.IO_Motor.setPower(0);
                }
            }


            if (gamepad1.y) {
                robot.IO_Servo_Left.setPosition(LeftServoOpen);
                robot.IO_Servo_Right.setPosition(RightServoOpen);
            }


            if (gamepad1.x) {
                robot.IO_Servo_Right.setPosition(RightServoOpen);
                robot.IO_Servo_Left.setPosition(LeftServoHalfOpen);
            }


            if (gamepad1.b) {
                robot.IO_Servo_Right.setPosition(RightServoHalfOpen);
                robot.IO_Servo_Left.setPosition(LeftServoOpen);
            }
            // Update telemetry
            robot.updateTelemetry();

        }

    }
}
