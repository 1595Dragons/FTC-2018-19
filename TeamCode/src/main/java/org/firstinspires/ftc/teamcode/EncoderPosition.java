package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Stephen Ogden on 12/6/18.
 * FTC 6128 | 7935
 * FRC 1595
 */

@TeleOp(name = "Find encoder positions", group = "Test")
public class EncoderPosition extends LinearOpMode {
    Config robot = new Config(this);

    @Override
    public void runOpMode() {
        boolean rightSelected = false;
        double position = 0.0;
        robot.ConfigureRobtHardware(false);
        waitForStart();
        while (opModeIsActive()) {
            if ((gamepad1.dpad_up || gamepad2.dpad_up) && position < 1) {
                position += 0.05;
            } else if ((gamepad1.dpad_up || gamepad2.dpad_up) && position > 0) {
                position -= 0.05;
            }

            if (gamepad1.dpad_left || gamepad1.dpad_right || gamepad2.dpad_right || gamepad2.dpad_left) {
                rightSelected = !rightSelected;
            }

            if (rightSelected) {
                robot.IO_Servo_Right.setPosition(position);
            } else {
                robot.IO_Servo_Left.setPosition(position);
            }

            telemetry.addData("Selected servo", rightSelected ? "Right servo" : "Left servo");
            telemetry.addData("Servo position", position);
            telemetry.update();
        }
    }
}
