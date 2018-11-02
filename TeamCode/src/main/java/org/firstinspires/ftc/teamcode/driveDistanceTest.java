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
        robot.setupForAuto();

        int error = 10;

        waitForStart();
        while (opModeIsActive()) {

            robot.driveDistance(MecanumDriveDirection.LEFT, 24, .75);

            if (robot.isThere(error, robot.left1) || robot.isThere(error, robot.left2) || robot.isThere(error, robot.right1) || robot.isThere(error, robot.right2)) {
                stop();
            }
            robot.updateTelemetry();

        }

    }
}
