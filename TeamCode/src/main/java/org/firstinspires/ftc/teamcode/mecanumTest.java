package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Stephen Ogden on 9/13/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
@TeleOp(name = "Mecanum Test", group = "Test")
public class mecanumTest extends LinearOpMode {
    config robot = new config();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.telemetry, this.hardwareMap);
        waitForStart();
        while(opModeIsActive()) {

            // Use the old drive code from last year
            double turn = gamepad1.right_stick_x * gamepad1.right_stick_x, throttle = gamepad1.left_stick_y, slow, powL, powR, turnSpeed=0.7d, sensitivity=0.4d;
            if (gamepad1.right_bumper) {
                slow = 0.35f;
            } else {
                slow = 1;
            }
            if(gamepad1.right_stick_x > 0) {
                powL = (throttle - turnSpeed*turn);
                powR = (throttle + turnSpeed*turn);
            }

            else {
                powL = (throttle + turnSpeed*turn);
                powR = (throttle - turnSpeed*turn);
            }
            if (powR > 1.0) {
                powL -= sensitivity * (powR - 1.0);
                powR = 1.0;
            } else if (powL > 1.0) {
                powR -=  sensitivity * (powL - 1.0);
                powL = 1.0;
            } else if (powR < -1.0) {
                powL +=  sensitivity * (-1.0 - powR);
                powR = -1.0;
            } else if (powL < -1.0) {
                powR +=  sensitivity * (-1.0 - powL);
                powL = -1.0;
            }

            // Modify power distribution based on bumpers, for swerve effect
            if (gamepad1.right_bumper) {
                robot.right_front.setPower(slow * powR * -1);
                robot.left_front.setPower(slow * powR * -1);
                robot.right_back.setPower(slow * powR);
                robot.left_back.setPower(slow * powL);
            } else if (gamepad1.left_bumper) {
                robot.right_back.setPower(slow * powR * -1);
                robot.left_back.setPower(slow * powR * -1);
                robot.right_front.setPower(slow * powR);
                robot.left_front.setPower(slow * powL);
            } else {
                robot.right_front.setPower(slow * powR);
                robot.right_back.setPower(slow * powR);
                robot.left_front.setPower(slow * powL);
                robot.left_back.setPower(slow * powL);
            }

            robot.updateTelemetry(this.telemetry, this.gamepad1);

        }
    }
}
