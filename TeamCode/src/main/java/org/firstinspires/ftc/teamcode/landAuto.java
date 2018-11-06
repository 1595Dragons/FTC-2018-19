package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Stephen Ogden on 10/31/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
@Autonomous(name = "Just land", group = "Official")
public class landAuto extends LinearOpMode {

    private config robot = new config(this.telemetry);

    public void runOpMode() {

        robot.ConfigureRobot(this.hardwareMap);
        robot.setupForAuto();

        int stage = 0;

        waitForStart();
        while (opModeIsActive()) {
            switch (stage) {
                case 0:
                    robot.climber.setTargetPosition(robot.maxClimberPos);
                    robot.climber.setPower(1);
                    stage++;
                    break;
                case 1:
                    if (robot.isThere(5, robot.climber)) {
                        robot.climber.setPower(0);
                        stage++;
                    }
                    break;
                case 2:
                    robot.driveDistance(MecanumDriveDirection.LEFT, 12, 1);
                    if (robot.isThere(15, robot.right1) || robot.isThere(15, robot.left1) || robot.isThere(15, robot.left2) || robot.isThere(15, robot.right2)) {
                        stage++;
                    }
                    break;
                case 3:
                    robot.zeroEncoderForMotors(robot.left2, robot.left1, robot.right1, robot.right2);
                    break;
                case 4:
                    stop();
                    break;

            }
            robot.updateTelemetry();
        }

    }
}
