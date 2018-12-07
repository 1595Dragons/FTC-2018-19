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
        double LeftServoClose = 0.1d, LeftServoOpen = 0.7d;
        double RightServoClose = 0.85d, RightServoOpen = 0.25d;


        //Motor Power
        double speedForTurn = 0.4, speedForMove = 0.5, speedForSide = 0.7, intakePower = 1, armPower = 1, extendPower = 0.8;


        double allPower, armExtend;

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


            // Send calculated power to wheels
            robot.left_front.setPower(Range.clip((-driveRightSide + driveForward + turnRight) * allPower, -1.0, 1.0));
            robot.right_front.setPower(Range.clip((driveRightSide + driveForward - turnRight) * allPower, -1.0, 1.0));
            robot.left_back.setPower(Range.clip((driveRightSide + driveForward + turnRight) * allPower, -1.0, 1.0));
            robot.right_back.setPower(Range.clip((-driveRightSide + driveForward - turnRight) * allPower, -1.0, 1.0));


            robot.armMotorL.setPower((gamepad1.left_stick_y) * armPower);
            robot.armMotorR.setPower((gamepad1.left_stick_y) * armPower);


            armExtend = (gamepad1.dpad_up) ? extendPower : (gamepad1.dpad_down ? -extendPower : 0);


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
                robot.IO_Servo_Left.setPosition(Math.round((LeftServoOpen + LeftServoClose) / 2));
            }


            if (gamepad1.b) {
                robot.IO_Servo_Right.setPosition(Math.round((RightServoOpen + RightServoClose) / 2));
                robot.IO_Servo_Left.setPosition(LeftServoOpen);
            }

            // Update telemetry with the gyro angles
            telemetry.addData("Lf power", robot.left_front.getPower())
                    .addData("Rf power", robot.right_front.getPower())
                    .addData("Lb power", robot.left_back.getPower())
                    .addData("Rb power", robot.right_back.getPower());
            telemetry.update();

        }
    }
}
