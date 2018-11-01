package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Stephen Ogden on 11/1/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
@Autonomous(name = "Drive distance funcion test", group = "Test")
public class driveDistanceTest extends LinearOpMode {

    private config robot = new config(this.telemetry);

    @Override
    public void runOpMode() {


        robot.ConfigureRobot(this.hardwareMap);


        waitForStart();
        while (opModeIsActive()) {

            robot.driveDistance(MecanumDriveDirection.FORWARD, 12, .5);
            if (robot.isThere(5, robot.left1) || robot.isThere(5, robot.left2) || robot.isThere(5, robot.right1) || robot.isThere(5, robot.right2)) {
                stop();
            }
            robot.updateTelemetry();

        }

    }
}
