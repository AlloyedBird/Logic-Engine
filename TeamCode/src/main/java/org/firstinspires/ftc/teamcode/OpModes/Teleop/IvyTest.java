package org.firstinspires.ftc.teamcode.OpModes.Teleop;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Geometry.Pose;
import org.firstinspires.ftc.teamcode.Localization.PoseEstimator;
import org.firstinspires.ftc.teamcode.Motion.MecanumKinematics;
import org.firstinspires.ftc.teamcode.Motion.PIDController;
import org.firstinspires.ftc.teamcode.Motion.WaypointFollower;
import org.firstinspires.ftc.teamcode.Pathing.PathCoordinator;
import org.firstinspires.ftc.teamcode.Vision.ShinyObjectDetector;

import java.util.ArrayList;

@TeleOp(name = "Ivy Scheduling Test", group = "Test")
public class IvyTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        // --- Hardware ---
        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // --- Subsystems ---
        PathCoordinator coordinator = new PathCoordinator(2); // inflation radius in cells
        coordinator.bakeStaticObstacles(new ArrayList<>());// TODO: add field elements

        ShinyObjectDetector shinyDetector = new ShinyObjectDetector();

        PoseEstimator estimator = new PoseEstimator(hardwareMap, new Pose(0, 0,0), shinyDetector);

        PIDController xPid       = new PIDController(0.1, 0, 0.01);
        PIDController yPid       = new PIDController(0.1, 0, 0.01);
        PIDController headingPid = new PIDController(0.1, 0, 0.01);

        WaypointFollower follower = new WaypointFollower(
                frontLeft, frontRight, backLeft, backRight,
                xPid, yPid, headingPid,
                coordinator, estimator
        );

        waitForStart();

        while (opModeIsActive()) {

            // --- Manual drive ---
            double x   =  gamepad1.left_stick_x;
            double y   = -gamepad1.left_stick_y;
            double rot =  gamepad1.right_stick_x;

            MecanumKinematics.WheelPowers wheels =
                    MecanumKinematics.calculate(x, y, rot);

            frontLeft.setPower(wheels.frontLeft);
            frontRight.setPower(wheels.frontRight);
            backLeft.setPower(wheels.backLeft);
            backRight.setPower(wheels.backRight);

            // --- Ivy tick ---
            Scheduler.execute();

            // --- Telemetry ---
            Pose pose = estimator.getPose();
            telemetry.addData("x",       pose.x);
            telemetry.addData("y",       pose.y);
            telemetry.addData("heading", Math.toDegrees(pose.heading));
            telemetry.update();

            // --- Update the obstacles for AStar ---
            coordinator.updateDynamicObstacles(new ArrayList<>());

        }

        estimator.stop();
    }
}