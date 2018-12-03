package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "all sensors", group = "Pushbot")
@Deprecated // Just use justTelemetry lol
@Disabled
public class sensors extends LinearOpMode {
    private Config robot = new Config(this);

    @Override
    public void runOpMode() {

        waitForStart();
        robot.ConfigureRobtHardware(false);

        while (opModeIsActive()) {
            telemetry.addData("encoder left front:%7d", robot.left_front.getCurrentPosition());
            telemetry.addData("encoder left back:%7d", robot.left_back.getCurrentPosition());
            telemetry.addData("encoder right front:%7d", robot.right_front.getCurrentPosition());
            telemetry.addData("encoder right back:%7d", robot.right_back.getCurrentPosition());

            telemetry.update();
        }

    }
}
