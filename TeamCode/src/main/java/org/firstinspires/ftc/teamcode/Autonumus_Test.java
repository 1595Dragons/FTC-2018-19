package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@Autonomous(name="Auto_test", group="Pushbot")

public class Autonumus_Test extends LinearOpMode {
    private config robot = new config(this.telemetry);

    @Override
        public void runOpMode (){
        robot.ConfigureRobtHardware(this.hardwareMap);
        ElapsedTime runtime = new ElapsedTime();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

            if (runtime.milliseconds() < 3000) {
                robot.right_front.setPower(0.5);
                robot.left_front.setPower(-0.5);
                robot.right_back.setPower(0.5);
                robot.left_back.setPower(-0.5);
            } else {

                //sleep(3000);
                robot.right_front.setPower(0);
                robot.left_front.setPower(0);
                robot.left_back.setPower(0);
                robot.right_back.setPower(0);

            }
        }
    }
}
