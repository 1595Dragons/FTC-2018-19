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

            // right stick is shifting left and right
            // left is standard drive

            robot.right_back.setPower(gamepad1.left_stick_y * gamepad1.left_stick_x);
            robot.right_front.setPower(gamepad1.left_stick_y * gamepad1.left_stick_x);
            robot.left_back.setPower(gamepad1.left_stick_y * gamepad1.left_stick_x);
            robot.left_front.setPower(gamepad1.left_stick_y * gamepad1.left_stick_x);

        }
    }
}
