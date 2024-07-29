package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "Teleop Robo-Centric")
@Config
public class TeleOpRoboCentric extends LinearOpMode {

    double speed;
    final ElapsedTime loopTime = new ElapsedTime();
    boolean pixelInOuttake; //need to use later
    final Gamepad currentGamepad1 = new Gamepad();
    final Gamepad currentGamepad2 = new Gamepad();
    final Gamepad previousGamepad1 = new Gamepad();
    final Gamepad previousGamepad2 = new Gamepad();

    boolean showMotorTelemetry = true;
    boolean showLoopTimes = true;
    boolean showPoseTelemetry = true;

    //power is 0.75
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime retractionTimer = new ElapsedTime();



    DcMotorEx leftFront;
    DcMotorEx rightFront;
    DcMotorEx leftBack;
    DcMotorEx rightBack;




    @Override
    public void runOpMode() {


        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Telemetry Init
        telemetry.setMsTransmissionInterval(50);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //TODO: REMEMBER TO COMMENT OUT BEFORE COMP

        // Motor Init








        // if(Math.abs(VariableStorage.leftLinearSlidesPos) > 20 || Math.abs(VariableStorage.rightLinearSlidesPos) > 20){
        //vertPIDController(0);
        // }
        // if(VariableStorage.dumperPos != dumpInitPosition){
        //  dumpServo1.setPosition(dumpInitPosition);
        //   dumpServo2.setPosition(dumpInitPosition);
        // }


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        waitForStart();

        if (isStopRequested()) return;


        // Run Period


        while (opModeIsActive() && !isStopRequested()) {
            loopTime.reset();
//            allHubs.forEach(LynxModule::clearBulkCache);

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            currentGamepad1.reset();
            currentGamepad2.reset();

            // CONTROLS

            // Gamepad 1
            // Driving Modifiers
            boolean padSlowMode = gamepad1.left_bumper;
            boolean padFastMode = gamepad1.right_bumper;
            boolean padResetPose = gamepad1.dpad_left && !previousGamepad1.dpad_left;

            if(gamepad1.left_trigger > 0) {
                leftFront.setPower(-0.3);
                leftBack.setPower(0.3);
                rightFront.setPower(0.3);
                rightBack.setPower(-0.3);
            }

            if(gamepad1.right_trigger > 0) {
                leftFront.setPower(0.3);
                leftBack.setPower(-0.3);
                rightFront.setPower(-0.3);
                rightBack.setPower(0.3);
            }
            // Update the speed
            if (padSlowMode) {
                speed = .25;
            } else if (padFastMode) {
                speed = 1;
            } else {
                speed = .9;
            }
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * speed,
                            -gamepad1.left_stick_x * speed
                    ),
                    -gamepad1.right_stick_x * speed
            ));

            drive.updatePoseEstimate();

            TelemetryPacket packet = new TelemetryPacket();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            double loopTimeMs = loopTime.milliseconds();

            if (showPoseTelemetry) {
                telemetry.addLine("--- Pose ---");
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", drive.pose.heading.toDouble());
            }
            if (showLoopTimes) {
                telemetry.addLine("--- Loop Times ---");
                telemetry.addData("loopTimeMs", loopTimeMs);
                telemetry.addData("loopTimeHz", 1000.0 / loopTimeMs);
            }

            telemetry.update();
        }

    }

}
