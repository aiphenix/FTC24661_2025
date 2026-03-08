package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;


@TeleOp
public class tele_blue extends LinearOpMode {
    ftc_2025_functions ftc_fns = new ftc_2025_functions();
    private ElapsedTime zeroGateTimer = new ElapsedTime();

    // Private variables
    double shoot_trigger = 0;
    double spitout_trigger = 0;
    boolean intake_toggle = false;
    boolean shoot_toggle = false;
    boolean gate_zeroed = false;
    private PathChain parkingPath;

    // Pedro pathing
    private Follower follower;
    public static Pose startPose;
    private boolean automatedDrive;

    public Limelight lime;

    @Override
    public void runOpMode()  {
        // Define Devices
        DcMotor FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        DcMotor FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        DcMotor BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        DcMotor BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        DcMotorEx ShootLeft = hardwareMap.get(DcMotorEx.class, "ShootLeft");
        DcMotorEx ShootRight = hardwareMap.get(DcMotorEx.class, "ShootRight");
        DcMotor Intake = hardwareMap.get(DcMotor.class, "Intake");
        DcMotor Gate = hardwareMap.get(DcMotor.class, "Gate");
        Servo HoodLeft = hardwareMap.get(Servo.class, "HoodLeft");
        Servo HoodRight = hardwareMap.get(Servo.class, "HoodRight");

        VoltageSensor ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        lime = new Limelight(hardwareMap, 8) ;

        List<DcMotor> allMotors = Arrays.asList(
                FrontLeft, FrontRight, BackLeft, BackRight
        );
        allMotors.forEach(motor -> {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        });

        ftc_fns.set_motor_orientations_PIDF_and_zero_power_behavior(
                FrontLeft, FrontRight, BackLeft, BackRight,
                ShootLeft, ShootRight, Intake, Gate, HoodLeft, HoodRight
        );

        // Set up telemetry
        telemetry.setMsTransmissionInterval(3);

        // adjust power based on voltage
        double curr_vol = ControlHub_VoltageSensor.getVoltage();
        double power_adj = ftc_fns.adjust_power(curr_vol, telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(
                startPose == null ? new Pose(120.5, 131.685, Math.toRadians(0)) : startPose);
        follower.update();

        parkingPath = follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(48.98, 182.18))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(-90), 0.8))
                .build();
        follower.update();

        waitForStart();

        // Initial gate zeroing (in case auton stopped at a point where gate is down)
        Gate.setPower(ftc_fns.init_gate_lift_pwr * power_adj); // Initial gate lift with low power
        zeroGateTimer.reset();

        follower.startTeleOpDrive();

        while (opModeIsActive()) {
            follower.update();

            // Initial gate zeroing (in case auton stopped at a point where gate is down)
            if (!gate_zeroed) {
                if (zeroGateTimer.milliseconds() >= 1500) {
                    ftc_fns.zero_gate(Gate);
                    ftc_fns.close_gate(Gate);
                    gate_zeroed = true;
                }
            }

            if (!automatedDrive) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x * 1.2, // Strafe correction
                        -gamepad1.right_stick_x,
                        false
                );
            }

            if (gamepad1.dpad_left) { // rezero heading for field centric driving
                Pose currentPose = follower.getPose();
                // Set current position to current X/Y but 0 Rotation
                follower.setPose(new Pose(currentPose.getX(), currentPose.getY(), 0));
            }

            // Intake motion
            spitout_trigger = gamepad1.left_trigger;
            shoot_trigger = gamepad1.right_trigger;

            // Left trigger to backspin intake
            if (spitout_trigger > 0) {
                Intake.setPower(-spitout_trigger);
                sleep(100);
                Intake.setPower(0);
            }

            // right trigger to manually shoot when aiming doesn't work
            if (shoot_trigger > 0) {
                if (Gate.getCurrentPosition() > 50) {
                    ftc_fns.lift_gate(true, Gate, Intake);
                }
                Intake.setPower(ftc_fns.shoot_trigger_intake_pwr);
                sleep(600);
                Intake.setPower(0);
                sleep(50); // wait till intake stops spinning and balls all leave
                ftc_fns.close_gate(Gate);
            }

            // Left toggle to spin up/down shooter
            if (gamepad1.leftBumperWasPressed()) {
                shoot_toggle = ! shoot_toggle;
                if (ShootLeft.getVelocity() > ftc_fns.convert_rpm_to_tps(ftc_fns.near_shot_shooter_rpm * 0.8)
                        | ShootRight.getVelocity() > ftc_fns.convert_rpm_to_tps(ftc_fns.near_shot_shooter_rpm * 0.8)) {
                    ftc_fns.set_shooter_speed(
                            0, false, ShootLeft, ShootRight, telemetry, gamepad1);
                    shoot_toggle = false;
                }
                if (shoot_toggle) { // speed up shooter wheel to near shot rpm
                    ftc_fns.set_shooter_speed(
                            ftc_fns.near_shot_shooter_rpm, true, ShootLeft, ShootRight, telemetry, gamepad1);
                } else { // stop shooter wheel
                    ftc_fns.set_shooter_speed(
                            0, false, ShootLeft, ShootRight, telemetry, gamepad1);
                }
            }
            // Right toggle to spin up/down intake
            if (gamepad1.rightBumperWasPressed()) {
                intake_toggle = ! intake_toggle;
                if (intake_toggle) {
                    if (Gate.getCurrentPosition() < ftc_fns.gate_down_position - 100) {
                        gamepad1.rumbleBlips(600);
                    } else {
                        Intake.setPower(ftc_fns.ball_pickup_intake_pwr * power_adj);
                    }
                } else {
                    Intake.setPower(0.2);
                }
            }

            // y to set shooter to near shot speed (to be used when manually shooting)
            if (gamepad1.yWasPressed()) {
                ftc_fns.near_shot_hood_servo_pos += 0.01;
                ftc_fns.far_shot_hood_servo_pos += 0.01;
                telemetry.addData("near Shot hood angle", ftc_fns.near_shot_hood_servo_pos);
                telemetry.addData("far Shot hood angle", ftc_fns.far_shot_hood_servo_pos);
                telemetry.update();
            }

            // b to set shooter to far sh ot speed (to be used when manually shooting)
            if (gamepad1.bWasPressed()) {
                ftc_fns.near_shot_hood_servo_pos -= 0.01;
                ftc_fns.far_shot_hood_servo_pos -= 0.01;
                telemetry.addData("near Shot hood angle", ftc_fns.near_shot_hood_servo_pos);
                telemetry.addData("far Shot hood angle", ftc_fns.far_shot_hood_servo_pos);
                telemetry.update();
            }

            // Manually zero gate
            if (gamepad1.xWasPressed()) {
                Gate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Gate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Gate.setDirection(DcMotor.Direction.FORWARD);
                Gate.setPower(ftc_fns.init_gate_lift_pwr * power_adj);
                sleep(1500);
                ftc_fns.zero_gate(Gate);
                ftc_fns.close_gate(Gate);
            }

            // Gate DOWN
            if (gamepad1.dpadDownWasPressed()) {
                ftc_fns.near_shot_shooter_rpm -= 50;
                ftc_fns.far_shot_shooter_rpm -= 50;
                telemetry.addData("near shot shooter rpm", ftc_fns.near_shot_shooter_rpm);
                telemetry.addData("far shot shooter rpm", ftc_fns.far_shot_shooter_rpm);
                telemetry.update();
                // ftc_fns.close_gate(Gate);
            } else if (gamepad1.dpadUpWasPressed()) {
                ftc_fns.near_shot_shooter_rpm += 50;
                ftc_fns.far_shot_shooter_rpm += 50;
                telemetry.addData("near shot shooter rpm", ftc_fns.near_shot_shooter_rpm);
                telemetry.addData("far shot shooter rpm", ftc_fns.far_shot_shooter_rpm);
                telemetry.update();

                // ftc_fns.lift_gate(true, Gate, Intake);
            }

            // Shooting
            if (gamepad1.aWasPressed() ) {
                Intake.setPower(0);
                int near_shot = ftc_fns.is_near_2(lime, telemetry);
                if (near_shot == -1) {
                    telemetry.addLine("Cannot tell if near or far shot. Abort...!");
                } else {
                    telemetry.addData("Near Shot 1:", near_shot);

                    if (Gate.getCurrentPosition() < ftc_fns.gate_down_position - 100) {
                        // close gate if not already, to prevent balls from accidentally being
                        // shot out when shooter is spinning up/down
                        ftc_fns.close_gate(Gate);
                    }

                    if (near_shot == 1) {
                        HoodLeft.setPosition(ftc_fns.near_shot_hood_servo_pos);
                        HoodRight.setPosition(ftc_fns.near_shot_hood_servo_pos);
                        ftc_2025_functions.AimResultPair result_pair = ftc_fns.auto_aim(
                                true, true, FrontLeft, FrontRight, BackLeft, BackRight,
                                lime, telemetry, gamepad1);
                        boolean aimed = result_pair.isSuccess();

                        if (aimed) {
                            double dist = ftc_fns.get_dist_safe(lime, true);
                            telemetry.addData("Near shot aim succeeded - distance", dist);
                            ftc_fns.set_shooter_speed(
                                    ftc_fns.near_shot_shooter_rpm * 0.7 * dist / (Math.sqrt(dist - 0.5)),
                                    true, ShootLeft, ShootRight, telemetry, gamepad1);
                            ftc_fns.make_near_shot(power_adj, true, true, Intake, Gate);
                            // Slow down shooter but not to 0
                            ShootLeft.setVelocityPIDFCoefficients(ftc_fns.pidf_p/4, 0, 0, ftc_fns.pidf_f/4);
                            ShootRight.setVelocityPIDFCoefficients(ftc_fns.pidf_p/4, 0,  0, ftc_fns.pidf_f/4);
                            ShootLeft.setPower(0.4);
                            ShootRight.setPower(0.4);
                            ShootLeft.setVelocityPIDFCoefficients(ftc_fns.pidf_p, 0, 0, ftc_fns.pidf_f);
                            ShootRight.setVelocityPIDFCoefficients(ftc_fns.pidf_p, 0, 0, ftc_fns.pidf_f);
                            HoodLeft.setPosition(ftc_fns.near_shot_hood_servo_pos);
                            HoodRight.setPosition(ftc_fns.near_shot_hood_servo_pos);
//                            ftc_fns.power_down_shooter(ShootLeft, ShootRight);
                        } else {
                            gamepad1.rumble(100);
                        }
                    } else if (near_shot == 0) { // far shot
                        HoodLeft.setPosition(ftc_fns.far_shot_hood_servo_pos);
                        HoodRight.setPosition(ftc_fns.far_shot_hood_servo_pos);
                        ftc_2025_functions.AimResultPair result_pair = ftc_fns.auto_aim(
                                true, true, FrontLeft, FrontRight, BackLeft, BackRight,
                                lime, telemetry, gamepad1);
                        boolean aimed = result_pair.isSuccess();
                        if (aimed) {
                            double dist = ftc_fns.get_dist_safe(lime, false);
                            telemetry.addData("Far shot aim succeeded - distance", dist);
                            ftc_fns.set_shooter_speed(
//                                    ftc_fns.far_shot_shooter_rpm * Math.sqrt(dist/2.9),
                                    ftc_fns.far_shot_shooter_rpm * 0.65 * dist / Math.sqrt(dist - 1),
                                    true, ShootLeft, ShootRight, telemetry, gamepad1);
                            ftc_fns.make_far_shot(power_adj, true, true, Intake, Gate);
                            // Slow down shooter but not to 0
                            ShootLeft.setVelocityPIDFCoefficients(ftc_fns.pidf_p/4, 0, 0, ftc_fns.pidf_f/4);
                            ShootRight.setVelocityPIDFCoefficients(ftc_fns.pidf_p/4, 0,  0, ftc_fns.pidf_f/4);
                            ShootLeft.setPower(0.4);
                            ShootRight.setPower(0.4);
                            ShootLeft.setVelocityPIDFCoefficients(ftc_fns.pidf_p, 0, 0, ftc_fns.pidf_f);
                            ShootRight.setVelocityPIDFCoefficients(ftc_fns.pidf_p, 0, 0, ftc_fns.pidf_f);
//                            ftc_fns.power_down_shooter(ShootLeft, ShootRight);
                        } else {
                            telemetry.addLine("Aiming Failed!");
                        }
                    }
                }
                // Uncomment next line to see above telemetry data. Comment out by default
                // to allow message from auto_aim() to remain on telemetry.
//                telemetry.update();
            }

            // Automated PathFollowing
            if (gamepad1.dpadRightWasPressed()) {
                follower.followPath(parkingPath, 1, true);
                automatedDrive = true;
            }

            // Stop automated following if the follower is done, or if X is pressed
            if (automatedDrive && (gamepad1.backWasPressed() || !follower.isBusy())) {
                follower.startTeleopDrive();
                automatedDrive = false;
            }

//             Monitoring (comment out for message from auto_aim() to remain on telemetry)
//            telemetry.addData("Left shooter speed", ShootLeft.getVelocity());
//            telemetry.addData("Right shooter speed", ShootRight.getVelocity());
//            double dist = lime.getDistance();
//            telemetry.addData("Distance", dist);
//            telemetry.addData("position", follower.getPose());
//            telemetry.update();
        }
    }
}