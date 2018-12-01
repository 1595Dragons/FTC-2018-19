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

    private RobotConfig robot = new RobotConfig(this.telemetry);

    public void runOpMode() {

        robot.configureRobot(this.hardwareMap);
        robot.resetMotors(robot.left1, robot.right1, robot.left2, robot.right2, robot.climber);

        int stage = 0;

        waitForStart();
        while (opModeIsActive()) {
            switch (stage) {
                case 0:
                    robot.climber.setTargetPosition(robot.maxClimberPos);
                    robot.climber.setPower(1);
                    if (robot.isThere(5, robot.climber)) {
                        robot.climber.setPower(0);
                        stage++;
                    }
                    break;
                case 1:
                    robot.driveDistance(MecanumDriveDirection.RIGHT, 4, 1);
                    if (robot.isThere(10, robot.right1, robot.left1, robot.left2, robot.right2)) {
                        robot.resetMotors(robot.left2, robot.left1, robot.right1, robot.right2);
                        stage++;
                    }
                    break;
                case 2:
                    robot.climber.setTargetPosition(robot.minClimberPos);
                    robot.climber.setPower(1);
                    if (robot.isThere(5, robot.climber)) {
                        robot.climber.setPower(0);
                        stage++;
                    }
                    break;
                case 3:
                    robot.driveDistance(MecanumDriveDirection.FORWARD, 38, 1);
                    if (robot.isThere(10, robot.right1, robot.left1, robot.left2, robot.right2)) {
                        robot.resetMotors(robot.left2, robot.left1, robot.right1, robot.right2);
                        stage++;
                    }
                    break;
                case 4:
                    stop();
                    break;
            }

            telemetry.addData("Stage", stage);
            telemetry.addLine();
            robot.updateTelemetry();

            idle();
        }

    }
}
