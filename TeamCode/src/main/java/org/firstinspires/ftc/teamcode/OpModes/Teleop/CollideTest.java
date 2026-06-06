package org.firstinspires.ftc.teamcode.OpModes.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Geometry.Pose;
import org.firstinspires.ftc.teamcode.Localization.PoseEstimator;
import org.firstinspires.ftc.teamcode.Motion.MecanumKinematics;
import org.firstinspires.ftc.teamcode.Motion.PIDController;
import org.firstinspires.ftc.teamcode.Motion.WaypointFollower;
import org.firstinspires.ftc.teamcode.Pathing.PathCoordinator;
import org.firstinspires.ftc.teamcode.Vision.ShinyObjectDetector;
import java.util.ArrayList;

@TeleOp(name = "Collision avoidance test", group = "test")
    public class CollideTest extends LinearOpMode{
        @Override
        public void runOpMode(){
            DcMotor frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
            DcMotor frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
            DcMotor backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
            DcMotor backRight = hardwareMap.get(DcMotorEx.class, "backRight");

            PathCoordinator coordinator = new PathCoordinator(2);
            coordinator.bakeStaticObstacles(new ArrayList<>());

            ShinyObjectDetector shinyDetector = new ShinyObjectDetector();
            PoseEstimator estimator = new PoseEstimator(hardwareMap,new Pose(0,0,0),shinyDetector);

            PIDController xPid = new PIDController(0.1,0,0.01);
            PIDController yPid = new PIDController(0.1, 0, 0.01);
            PIDController headingPID = new PIDController(0.1,0,0.01);

            WaypointFollower follower = new WaypointFollower(frontLeft, frontRight, backLeft, backRight,
                                                                xPid, yPid, headingPID, coordinator, estimator);

            waitForStart();

            while (opModeIsActive()){
                double x = gamepad1.left_stick_x;
                double y = -gamepad1.left_stick_y;
                double rot = gamepad1.right_stick_x;

                MecanumKinematics.WheelPowers wheels = MecanumKinematics.calculate(x,y,rot);
                frontLeft.setPower(wheels.frontLeft);
                frontRight.setPower(wheels.frontRight);
                backLeft.setPower(wheels.backLeft);
                backRight.setPower(wheels.backRight);

                if (shinyDetector.isRobotDetected()){
                    telemetry.addData("Robot detected?", shinyDetector);
                } else {
                    telemetry.addData("No robot detected", null);
                }
            }
            estimator.stop();
        }
}