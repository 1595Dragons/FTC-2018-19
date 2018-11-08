package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Stephen Ogden on 11/8/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
@Autonomous(name = "Land and detect mineral", group = "Official")
public class LandAndDriveAuto extends LinearOpMode {

    private RobotConfig robot = new RobotConfig(this.telemetry);

    @Override
    public void runOpMode() {

        robot.configureRobot(this.hardwareMap);
        robot.setupGoldDetector(this.hardwareMap);
        robot.resetMotors(robot.left1, robot.right1, robot.left2, robot.right2, robot.climber);

        int stage = 0;

        waitForStart();
        while (opModeIsActive()) {

            switch (stage) {
                case 0:
                    robot.climber.setTargetPosition(robot.maxClimberPos);
                    robot.climber.setPower(1);

                    if (robot.isThere(1, robot.climber)) {
                        robot.climber.setPower(0);
                        stage++;
                    }

                    break;
                case 1:
                    robot.driveDistance(MecanumDriveDirection.FORWARD, 8, 1);
                    if (robot.isThere(5, robot.right1, robot.left1, robot.left2, robot.right2)) {
                        robot.resetMotors(robot.left2, robot.left1, robot.right1, robot.right2);
                        stage++;
                    }
                    break;
                case 2:
                    robot.climber.setTargetPosition(robot.minClimberPos);
                    robot.climber.setPower(1);

                    if (robot.isThere(1, robot.climber)) {
                        robot.climber.setPower(0);
                        stage++;
                    }
                    break;
                case 3:
                    // TODO: Move out from lander (slide right)
                    if (robot.isThere(5, robot.right1, robot.left1, robot.left2, robot.right2)) {
                        robot.resetMotors(robot.left2, robot.left1, robot.right1, robot.right2);
                        stage++;
                    }
                    break;
                case 4:
                    // TODO: Turn to face minerals
                    if (robot.isThere(5, robot.right1, robot.left1, robot.left2, robot.right2)) {
                        robot.resetMotors(robot.left2, robot.left1, robot.right1, robot.right2);
                        stage++;
                    }
                    break;
                case 5:
                    robot.goldDetector.enable();
                    // TODO: Go through each mineral
                    if (robot.goldDetector.isFound()) {
                        stage++;
                    }
                    break;
                case 6:
                    robot.goldDetector.disable();
                    // TODO: Drive forward
                    stage++;
                    break;
                case 7:
                    stop();
                    break;
            }

        }

        robot.updateTelemetry();

    }
}
