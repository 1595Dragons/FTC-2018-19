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

    private RobotConfig robot = new RobotConfig(this.telemetry);

    @Override
    public void runOpMode() {

        robot.configureRobot(this.hardwareMap);
        //robot.setupForAuto();
        robot.resetMotors(robot.left1, robot.right2, robot.left2, robot.right1);

        int error = 5;

        waitForStart();
        while (opModeIsActive()) {

            robot.driveDistance(MecanumDriveDirection.DIAGUPLEFT, 24, .75);

            if (robot.isThere(error, robot.left1, robot.left2, robot.right1, robot.right2)) {
                stop();
            }
            robot.updateTelemetry();

        }

    }
}
