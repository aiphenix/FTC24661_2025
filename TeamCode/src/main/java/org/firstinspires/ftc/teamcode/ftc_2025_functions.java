// This entire codes if at 13.7V batter
package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.pedropathing.util.Timer;
import java.time.Duration;
import java.time.Instant;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Limelight;

import java.util.ArrayList;
import java.util.List;

public class ftc_2025_functions extends LinearOpMode {
    private Limelight3A limelight;
    private Servo HoodLeft;
    private Servo HoodRight;
    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor BackLeft;
    private DcMotorEx ShootRight;
    private DcMotorEx ShootLeft;
    private VoltageSensor ControlHub_VoltageSensor;

    // --------------- Constants --------------

    // Initialization
    public double init_gate_lift_pwr = -0.2;
    long total_init_gate_lift_time = 3500;
    double auton_init_move_back_pwr = 0.5;
    long auton_init_move_back_time = 1700;
    public int gate_down_position  = 490;

    // Shooting
    public double near_shot_hood_servo_pos = 0.6;
    public double far_shot_hood_servo_pos = 0.68;
    public double near_shot_shooter_rpm = 2450;
    public double far_shot_shooter_rpm = 3600;
    public double shoot_trigger_intake_pwr = 1; // TODO: Can we turn it down to 0.9?

    // Movement
    public double wheel_pwr = 1;
    public double ball_intake_move_power_adj = 0.7;
    public double ball_pickup_intake_pwr = 0.65; // TODO: Can we tune down to 0.5?
    public long auton_time_to_leave_near_shot_area = 650;

    // Spike 1
    int rot_time1 = 170;
    int strafe_time1 = 525; // 525 is too little, 575 is too much
    int intake_time1 = 750; // 6725 is otherwise perfect but kicks the 3rd remaining ball a little too hard.
    int strafe_back_time1 = 485;
    int rot_back_time1 = 170;

    // Spike 2
    int rot_time2 = rot_time1; // TODO: should it be different from rot_time1?
    int strafe_time2 = 1050;
    int intake_time2 = intake_time1;
    int strafe_back_time2 = 950;
    int rot_back_time2 = rot_back_time1;

    // Spike 3
    int rot_time3 = rot_time1+10; // TODO: should it be different from rot_time1?
    int strafe_time3 = 1650; // 1600 is too much, 1550 is not enough
    int intake_time3 = intake_time1;
    int strafe_back_time3 = 1550;
    int rot_back_time3 = rot_back_time1+20;

    // Spike 1 red
    int rot_time1_red = rot_time1+32;
    int strafe_time1_red = strafe_time1+15;
    int intake_time1_red = intake_time1;
    int strafe_back_time1_red = strafe_back_time1;
    int rot_back_time1_red = rot_time1_red;

    // Spike 2 red
    int rot_time2_red = rot_time2+37;
    int strafe_time2_red = strafe_time2+50;
    int intake_time2_red = intake_time2;
    int strafe_back_time2_red = strafe_back_time2+25;
    int rot_back_time2_red = rot_time2_red;

    // Spike 3 red
    int rot_time3_red = rot_time3+60;
    int strafe_time3_red = strafe_time3-50;
    int intake_time3_red = intake_time3;
    int strafe_back_time3_red = strafe_back_time3;
    int rot_back_time3_red = rot_time3_red;

    // --------------- Functions --------------

    public ftc_2025_functions(HardwareMap hardwareMap) {

    }

    // Systems functions
    public double adjust_power(double curr_vol, Telemetry telemetry) {
        // double power_adj = 1.04; // > 13v : 1.08 | 12.5-13v : 1.04 | 12.25-12.5v : 1.02 | 12-12.25 : 1.0 | 11.8: 0.98 (<12v CHANGE BATTERY)
        double power_adj;
        if (curr_vol >= 13) {
            power_adj = 0.9;
        } else if (curr_vol >= 12.5) {
            power_adj = 0.96;
        } else if (curr_vol >= 12.25) {
            power_adj = 1.02;
        } else if (curr_vol > 12) {
            power_adj = 1.06;
        } else {
            power_adj = 1.06;
        }
        telemetry.addData("Battery Voltage Sensed", curr_vol);
        telemetry.addData("Power Adjustment", power_adj);
        telemetry.update();
        return power_adj;
    }

    public void set_motor_orientations_PIDF_and_zero_power_behavior(
            DcMotor FrontLeft, DcMotor FrontRight, DcMotor BackLeft, DcMotor BackRight,
            DcMotorEx ShootLeft, DcMotorEx ShootRight, DcMotor Intake, DcMotor Gate,
            Servo HoodLeft, Servo HoodRight
    ) {
        // Orient motors
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.FORWARD);

        ShootLeft.setDirection(DcMotorEx.Direction.FORWARD);
        ShootRight.setDirection(DcMotorEx.Direction.REVERSE);

        Intake.setDirection(DcMotor.Direction.FORWARD);
        Gate.setDirection(DcMotor.Direction.FORWARD);
        HoodLeft.setDirection(Servo.Direction.FORWARD);
        HoodRight.setDirection(Servo.Direction.REVERSE);

        // Set motor zero power behavior
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShootLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        ShootRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set shooter PIDF coefs
        ShootRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShootLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShootLeft.setVelocityPIDFCoefficients(125, 0, 0, 16.8);
        ShootRight.setVelocityPIDFCoefficients(125, 0, 0, 16.8);
    }

    // gate function
    public void lift_gate(boolean wait_till_completion, DcMotor Gate, DcMotor Intake) {
        // stop intake in case it is still forward spinning.
        Intake.setPower(0);
        sleep(50);
        // unroll the intake to release pressure on gate
        Intake.setPower(-0.2);
        sleep(80);
        Intake.setPower(0);

        // Lift gate
        Gate.setTargetPosition(0);

        if (wait_till_completion) {
            while (Gate.getCurrentPosition() > 0) {
                sleep(100);
            }
        }
    }
    public void close_gate(DcMotor Gate) {
        Gate.setTargetPosition(gate_down_position); // TODO: changed from 1350 to prevent gate snapping during auton - does it help?
    }
    public void zero_gate(DcMotor Gate) {
        // TODO: backspin motor by a tiny bit to release pressure
        Gate.setPower(0.1);
        sleep(50);
        Gate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Gate.setPower(1);
        Gate.setTargetPosition(0);
        Gate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Movement Functions
    public void rotate_counter_clockwise(
            DcMotor FrontLeft, DcMotor FrontRight, DcMotor BackLeft, DcMotor BackRight,
            double wh_pw) {
        FrontLeft.setPower(-wh_pw);
        FrontRight.setPower(wh_pw);
        BackLeft.setPower(-wh_pw);
        BackRight.setPower(wh_pw);
    }
    public void rotate_clockwise(
            DcMotor FrontLeft, DcMotor FrontRight, DcMotor BackLeft, DcMotor BackRight,
            double wh_pw) {
        rotate_counter_clockwise(FrontLeft, FrontRight, BackLeft, BackRight, -wh_pw);
    }
    public void stop_drive(
            DcMotor FrontLeft, DcMotor FrontRight, DcMotor BackLeft, DcMotor BackRight,
            int post_sleep) {
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
        sleep(post_sleep);
    }
    public void move_forward(
            DcMotor FrontLeft, DcMotor FrontRight, DcMotor BackLeft, DcMotor BackRight,
            double wh_pw) {
        FrontLeft.setPower(wh_pw);
        FrontRight.setPower(wh_pw);
        BackLeft.setPower(wh_pw);
        BackRight.setPower(wh_pw);
    }
    public void move_back(
            DcMotor FrontLeft, DcMotor FrontRight, DcMotor BackLeft, DcMotor BackRight,
            double wh_pw) {
        move_forward(FrontLeft, FrontRight, BackLeft, BackRight, -wh_pw);
    }
    public void strafe_left(
            DcMotor FrontLeft, DcMotor FrontRight, DcMotor BackLeft, DcMotor BackRight,
            double wh_pw) {
        FrontLeft.setPower(-wh_pw);
        FrontRight.setPower(wh_pw);
        BackLeft.setPower(wh_pw);
        BackRight.setPower(-wh_pw);
    }
    public void strafe_right(
            DcMotor FrontLeft, DcMotor FrontRight, DcMotor BackLeft, DcMotor BackRight,
            double wh_pw) {
        strafe_left(FrontLeft, FrontRight, BackLeft, BackRight, -wh_pw);
    }

    // Limelight
    public List<Double> get_lime_reading(Limelight limelight, Telemetry telemetry,
                                         boolean bypass_sanity_check) {
        /*
        Keep reading lime and take the average of last 5 valid values.
        If it cannot find 5 values after 25 attempts (FPS = 65), abort.
            In that case, compute the average of available readings.
        If any of the readings is more than 10% deviated from the average, re-sample
        */
        int max_tries = 5_000_000;
        int n_readings_needed = 5;
        List<Double> txs = new ArrayList<>();
        List<Double> tys = new ArrayList<>();
        List<Double> tas = new ArrayList<>();
//        List<Pose> poses = new ArrayList<>();
        List<Double> final_values = new ArrayList<>();

        int total_read_counter = 0;
        while (txs.size() < n_readings_needed) {
            total_read_counter++;
            if (total_read_counter > max_tries) {
                break;
            }

            LLResult lime_result = limelight.getLatestResult();
            if (lime_result.isValid()) {
                txs.add(lime_result.getTx());
                tys.add(lime_result.getTy());
                tas.add(lime_result.getTa());
//                Pose pose = new Pose(
//                        lime_result.getBotpose_MT2().getPosition().x,
//                        lime_result.getBotpose_MT2().getPosition().y,
//                        lime_result.getBotpose_MT2().getOrientation().getYaw(AngleUnit.DEGREES));
//                poses.add(pose);
                // Pose3D pose =  lime_result.getBotpose(); // [ X, Y, Z, roll, pitch, yaw, latency ]
            }
        }
        telemetry.addData("Total Limelight Readings", total_read_counter);
        telemetry.update();
        if (!txs.isEmpty()) { // Has some readings, take average
            final_values.add(average(txs));
            final_values.add(average(tys));
            final_values.add(average(tas));
//            final_values.add(poses.get(poses.size()-1));
        } else {
            // final_values will all be an empty lists
        }
        if (! bypass_sanity_check) {
            boolean lime_reading_valid = check_lime_reading_is_valid(
                    final_values, txs, tys, tas, telemetry);
            if (!lime_reading_valid) {
                // Reading is not valid, attempt one more time but bypass sanity check
                final_values = get_lime_reading(limelight, telemetry,true);
            }
        }
        return final_values;
    }
    public boolean check_lime_reading_is_valid(
            List<Double> final_values, List<Double> txs,
            List<Double> tys, List<Double> tas, Telemetry telemetry) {
        double tolerance_thresh = 0.2; // 2 0% tolerance
        if (final_values.isEmpty()) {
            // empty list is always valid
            return true;
        } else {
            for (double value : txs) {
                if (Math.abs(value / final_values.get(0) - 1) > tolerance_thresh) {
                    return false;
                }
            }
            for (double value : tys) {
                if (Math.abs(value / final_values.get(1) - 1) > tolerance_thresh) {
                    return false;
                }
            }
            return true; // if neither x nor y checks triggers an error, return true
        }
    }
    public int is_near_3(Limelight limelight, Telemetry telemetry) {
        double dist = limelight.getDistance();
        if (dist == 0) {
            return -1; // bad read
        } else if (dist < 2) {
            return 1; // near shot
        } else {
            return 0; // far shot
        }
    }

    public int is_near_2(Limelight limelight, Telemetry telemetry) {
        List<Double> bot_pos = get_lime_reading(limelight, telemetry,false);
        if (bot_pos.isEmpty()) {
            telemetry.addData("Near / Far check", "FAILED!");
            telemetry.update();
            return -1; // return true because it doesn't matter as the bot can't aim anyway
        } else {
            telemetry.addData("Target Area %", bot_pos.get(2));
            telemetry.update();
            if (bot_pos.get(2) > 0.8) {
                return 1;
            } else {
                return 0;
            }
        }
    }

    public boolean is_near(Limelight limelight, Telemetry telemetry) {
        List<Double> bot_pos = get_lime_reading(limelight, telemetry,false);
        if (bot_pos.isEmpty()) {
            telemetry.addData("Near / Far check", "FAILED!");
            telemetry.update();
            return true; // return true because it doesn't matter as the bot can't aim anyway
        } else {
            telemetry.addData("Target Area %", bot_pos.get(2));
            telemetry.update();
            return bot_pos.get(2) > 0.8;
        }
    }

    public boolean auto_aim_x_only_sm(
            Boolean is_blue, DcMotor FrontLeft, DcMotor FrontRight, DcMotor BackLeft, DcMotor BackRight,
            Limelight limelight, Telemetry telemetry, Gamepad gamepad1, Follower follower
    ) {
        // Check if the shot is near or far4
        boolean near_shot = is_near(limelight, telemetry);

        // Set targets and power/time needed for incremental adjustment
        double target_x = 0.0;
        double x_tol = 3;
        double x_power = 0.3;

        if (! near_shot) { // far shooting
            if (is_blue) {
                target_x = -2;
            } else {
                target_x = 2;
            }
            x_tol = 0.5;
            x_power = 0.2;
        }

        List<Double> limeReading = get_lime_reading(limelight, telemetry, false);
        telemetry.addData("limeReading", limeReading);
        telemetry.update();
        if (! limeReading.isEmpty()) {
            telemetry.addLine("Entering Aiming");
            telemetry.update();
            double curr_x = limeReading.get(0);
            double curr_x_diff = curr_x - target_x;
            if (curr_x_diff < -x_tol) {
                telemetry.addLine("Rotate clockwise ->");
                telemetry.update();

                rotate_clockwise(FrontLeft, FrontRight, BackLeft, BackRight, x_power);
                return false;
            } else if (curr_x_diff > x_tol) {
                telemetry.addLine("Rotate counterclockwise <-");
                telemetry.update();

                rotate_counter_clockwise(FrontLeft, FrontRight, BackLeft, BackRight, x_power);
                return false;
            } else {
//                stop_drive(FrontLeft, FrontRight, BackLeft, BackRight, 0);
                telemetry.addLine("Auto Aim Success!");
                telemetry.update();
                return true;
            }
        } else {
            // Cannot get lime reading. Use heading to rotate
            double curr_heading = follower.getHeading();
            telemetry.addData("Current heading", curr_heading);
            telemetry.update();
            if (curr_heading > -45 && curr_heading < 135) {
                telemetry.addLine("Cannot see Tag - Rotate clockwise ->");
                telemetry.update();
                rotate_clockwise(FrontLeft, FrontRight, BackLeft, BackRight, x_power);
            }  else if (curr_heading < -60 || curr_heading >= 135) {
                telemetry.addLine("Cannot see Tag - Rotate counterclockwise <-");
                telemetry.update();
                rotate_counter_clockwise(FrontLeft, FrontRight, BackLeft, BackRight, x_power);
            } else {
                telemetry.addLine("In range, do not rotate");
                telemetry.update();
//                stop_drive(FrontLeft, FrontRight, BackLeft, BackRight, 0);
            }
            return false;
        }
    }

    public static class AimResultPair {
        private final boolean success;
        private final long time_elapsed;

        public AimResultPair(boolean success, long time_elapsed) {
            this.success = success;
            this.time_elapsed = time_elapsed;
        }

        public boolean isSuccess() {
            return success;
        }

        public long getTimeElapsed() {
            return time_elapsed;
        }
    }
    public AimResultPair auto_aim (
            Boolean is_blue, Boolean skip_y, DcMotor FrontLeft, DcMotor FrontRight, DcMotor BackLeft, DcMotor BackRight,
            Limelight limelight, Telemetry telemetry, Gamepad gamepad1) {
        // break before aim
        stop_drive(FrontLeft, FrontRight, BackLeft, BackRight,0);
        sleep(300); // This proves to be necessary for aim to work TODO: see if needs to be raised or lowered

        telemetry.addLine("Auto_aim started");
        telemetry.update();
//        Instant start = Instant.now();
        int auto_aim_x_speed_booster = 1;
        double auto_aim_max_x_speed_booster = 1;
        int auto_aim_y_speed_booster = 1;
        double auto_aim_max_y_speed_booster = 1;
        int max_aim_steps = 40;
        int max_x_y_tries = 70;

        // Check if the shot is near or far4
        boolean near_shot = is_near(limelight, telemetry);
        // Set targets and power/time needed for incremental adjustment
        double target_x = 0.0;
        double x_tol = 3;
        double x_power = 0.3;
        long x_drive_time = 20;

        double target_y = -11.6;
        double y_tol = 0.15;
        double y_power = 0.2;
        long y_drive_time = 25;

        if (! near_shot) { // far shooting
            if (is_blue) {
                target_x = -2;
            } else {
                target_x = 2;
            }
            x_tol = 0.5;
            x_power = 0.2;
            x_drive_time = 10;

            target_y = 7.9;
            y_tol = 0.1;
            y_power = 0.2;
            y_drive_time = 10;
        }

        boolean aimed = false;
        int fail_reason = 0;
        List<Double> limeReading = get_lime_reading(limelight, telemetry, false);
        if (! limeReading.isEmpty()) {
            telemetry.addLine("Entering Aiming");
            telemetry.update();

            int aim_steps_taken = 0;
            aimloop: while (! aimed) {
                telemetry.addData("Aiming step", aim_steps_taken);
                telemetry.update();

                limeReading = get_lime_reading(limelight, telemetry,false);
                if (limeReading.isEmpty()) {
                    fail_reason = 1;
                    telemetry.addData("Failed to read April Tag", "1");
                    telemetry.update();
                    break;
                }
                double curr_x = limeReading.get(0);
                double curr_x_diff = curr_x - target_x;
                long x_tries = 0;
                while (Math.abs(curr_x_diff) > x_tol) {
                    if (curr_x_diff < -x_tol) {
                        double power_boost = Math.min(
                                Math.max(auto_aim_y_speed_booster * (-curr_x_diff/y_tol), 1), auto_aim_max_x_speed_booster);
                        rotate_clockwise(FrontLeft, FrontRight, BackLeft, BackRight, x_power * power_boost);
                        sleep((long) (x_drive_time / power_boost));
                        stop_drive(FrontLeft, FrontRight, BackLeft, BackRight,0);
                    } else {
                        double power_boost = Math.min(
                                Math.max(auto_aim_y_speed_booster * (curr_x_diff/y_tol), 1), auto_aim_max_x_speed_booster);
                        rotate_counter_clockwise(FrontLeft, FrontRight, BackLeft, BackRight, x_power * power_boost);
                        sleep((long) (x_drive_time / power_boost));
                        stop_drive(FrontLeft, FrontRight, BackLeft, BackRight, 0);
                    }

                    limeReading = get_lime_reading(limelight, telemetry, false);
                    if (limeReading.isEmpty()) {
                        fail_reason = 2;
                        telemetry.addData("Failed to read April Tag", "2");
                        telemetry.update();
                        break aimloop;
                    }
                    curr_x = limeReading.get(0);
                    curr_x_diff = curr_x - target_x;
                    x_tries++;
                    if (x_tries > max_x_y_tries) {
                        break aimloop;
                    }
                    if (gamepad1.backWasPressed()) {
                        break aimloop;
                    }
                }

                limeReading = get_lime_reading(limelight, telemetry, false);
                if (limeReading.isEmpty()) {
                    fail_reason = 3;
                    telemetry.addData("Failed to read April Tag", "3");
                    telemetry.update();
                    break;
                }
                double curr_y = limeReading.get(1);
                double curr_y_diff = curr_y - target_y;
                long y_tries = 0;
                if (!skip_y) {
                    while (Math.abs(curr_y_diff) > y_tol) {
                        if (curr_y - target_y < -y_tol) {
                            double power_boost = Math.min(
                                    Math.max(auto_aim_y_speed_booster * (-curr_y_diff/y_tol), 1), auto_aim_max_y_speed_booster);
                            move_back(FrontLeft, FrontRight, BackLeft, BackRight,
                                    y_power * power_boost);
                            telemetry.addData("Y Power Multiplier", power_boost);
                            telemetry.update();

                            sleep((long) (y_drive_time/power_boost));
                            stop_drive(FrontLeft, FrontRight, BackLeft, BackRight, 0);
                        } else {
                            double power_boost = Math.min(
                                    Math.max(auto_aim_y_speed_booster * (curr_y_diff/y_tol), 1), auto_aim_max_y_speed_booster);
                            move_forward(FrontLeft, FrontRight, BackLeft, BackRight,
                                    y_power * power_boost);
                            telemetry.addData("Y Power Multiplier", power_boost);
                            telemetry.update();

                            sleep((long) (y_drive_time/power_boost));
                            stop_drive(FrontLeft, FrontRight, BackLeft, BackRight, 0);

                        }

                        limeReading = get_lime_reading(limelight, telemetry,false);
                        if (limeReading.isEmpty()) {
                            fail_reason = 4;
                            telemetry.addData("Failed to read April Tag", "4");
                            telemetry.update();
                            break aimloop;
                        }
                        curr_y = limeReading.get(1);
                        curr_y_diff = curr_y - target_y;
                        y_tries++;
                        if (y_tries > max_x_y_tries) {
                            break aimloop;
                        }
                        if (gamepad1.backWasPressed()) {
                            break aimloop;
                        }
                    }
                }

                // check if aimed
                limeReading = get_lime_reading(limelight, telemetry,false);
                if (limeReading.isEmpty()) {
                    fail_reason = 5;
                    telemetry.addData("Failed to read April Tag", "5");
                    telemetry.update();
                    break;
                }
                curr_x = limeReading.get(0);
                curr_y = limeReading.get(1);
                if (!skip_y && Math.abs(curr_x - target_x) <= x_tol && Math.abs(curr_y - target_y) <= y_tol) {
                    aimed = true;
                } else if (skip_y && Math.abs(curr_x - target_x) <= x_tol) {
                    aimed = true;
                }
                aim_steps_taken++;
                if (aim_steps_taken > max_aim_steps) {
                    break;
                }
                if (gamepad1.backWasPressed()) {
                    break;
                }
            }
        }
        telemetry.addLine("Aiming Done!");
        telemetry.update();
        if (! aimed) {
            telemetry.addData("Auto Aim [FINAL CHECK]", "Failed... ");
            telemetry.addData("Fail Reason", fail_reason);
            gamepad1.rumble(250);
            gamepad1.rumbleBlips(100);
        } else {
            telemetry.addData("Auto Aim [FINAL CHECK]", "Success!");
        }
        double dist = limelight.getLastDist();
        telemetry.addData("Distance", dist);
        telemetry.update();

//        Instant end = Instant.now();
//        Duration duration = Duration.between(start, end);
//        long time_elapsed = duration.toMillis();
        long time_elapsed = 300;
        return new AimResultPair(aimed, time_elapsed);
    }

    // Helper Functions
    public double average(List<Double> xs) {
        double running_sum = 0;
        for (double value : xs) {
            running_sum += value;
        }
        return running_sum / xs.size();
    }
    public double convert_rpm_to_tps(double rpm) {
        int ticks_per_round = 28;
        return rpm * ticks_per_round / 60;
    }

    // Shoot functions
    public void set_shooter_speed(
            double rpm, boolean wait_till_reach_tgt_rpm, DcMotorEx ShootLeft, DcMotorEx ShootRight,
            Telemetry telemetry, Gamepad gamepad1) {
        double tps = convert_rpm_to_tps(rpm);
        ShootLeft.setVelocity(tps);
        ShootRight.setVelocity(tps);

        if (wait_till_reach_tgt_rpm) {
            wait_till_shooter_reach_tgt_rpm(rpm, ShootLeft, ShootRight, telemetry, gamepad1);
        }
    }
    public void wait_till_shooter_reach_tgt_rpm (
            double rpm, DcMotorEx ShootLeft, DcMotorEx ShootRight, Telemetry telemetry,
            Gamepad gamepad1) {
        double tps = convert_rpm_to_tps(rpm);
        double shooter_left_act_vel = ShootLeft.getVelocity();
        double shooter_right_act_vel = ShootRight.getVelocity();
        while (Math.abs(shooter_left_act_vel / tps - 1) > 0.03
                || Math.abs(shooter_right_act_vel / tps - 1) > 0.03) {
            sleep(50); // TODO: Can we tune down to 50?
            shooter_left_act_vel = ShootLeft.getVelocity();
            shooter_right_act_vel = ShootRight.getVelocity();
            telemetry.addData("Shooter want speed", tps);
            telemetry.addData("Shooter left actual speed", shooter_left_act_vel);
            telemetry.addData("Shooter right actual speed", shooter_right_act_vel);
            telemetry.update();
            if (gamepad1.backWasPressed()) {
                break;
            }
        }
    }
    public void make_near_shot(double power_adj, boolean close_gate_after, DcMotor Intake, DcMotor Gate) {
        // lift gate
        lift_gate(true, Gate, Intake);

        // shoot
        Intake.setPower(shoot_trigger_intake_pwr * power_adj);
        sleep(1000); // TODO: Can we tune do wn to 1000?
        Intake.setPower(0);

        // close gate
        if (close_gate_after) {
            sleep(100); // sleep to be safe no ball is being pushed by intake TODO: tune down to 50?
            close_gate(Gate);
        }
    }
    public void make_far_shot(double power_adj, boolean close_gate_after, DcMotor Intake, DcMotor Gate) {
        // lift gate
        lift_gate(true, Gate, Intake);

        Intake.setPower(shoot_trigger_intake_pwr * power_adj);
        sleep(150); // TODO: Can we tune down to 1000?
        Intake.setPower(0);
        sleep (250);
        Intake.setPower(shoot_trigger_intake_pwr * power_adj);
        sleep(250); // TODO: Can we tune down to 1000?
        Intake.setPower(0);
        sleep (900);
        Intake.setPower(shoot_trigger_intake_pwr * power_adj);
        sleep(450); // TODO: Can we tune down to 1000?
        Intake.setPower(0);

        // close gate
        if (close_gate_after) {
            sleep(100); // sleep to be safe no ball is being pushed by intake TODO: tune down to 50?
            close_gate(Gate);
        }
    }
    public void power_down_shooter(DcMotorEx ShootLeft, DcMotorEx ShootRight) {
        ShootLeft.setPower(0);
        ShootRight.setPower(0);
    }

    // Shooting state machine
    public enum SHOOTING {
        PREWAIT,
        BACKSPIN,
        LIFT_GATE,
        INTAKE_FEED,
        CLOSE_GATE,
        FOLLOW
    }

    private boolean wasActionTimerReset = false;
    private Timer pathTimer, actionTimer;
    private int pathState;
    private SHOOTING s_states;

    public void waitThenFollowPath (
            double waitTime, PathChain nextPath, double pathMaxPower, int nextPathState,
            Follower follower, SHOOTING s_states) {
        if(!follower.isBusy()) {
            if (!wasActionTimerReset) {
                actionTimer.resetTimer();
                wasActionTimerReset = true;
            }

            if (actionTimer.getElapsedTimeSeconds() > waitTime) {
                follower.followPath(nextPath, pathMaxPower, true);
                s_states = SHOOTING.PREWAIT;
                wasActionTimerReset = false;
                setPathState(nextPathState, pathTimer);
            }
        }
    }

    public void shootThenFollowPath (
            double preWaitTime, PathChain nextPath, double pathMaxPower, int nextPathState,
            double power_adj,
            Follower follower, SHOOTING s_states, DcMotor Intake, DcMotor Gate) {
        actionTimer = new Timer();
        if(!follower.isBusy()) {
            switch (s_states) {
                case PREWAIT:
                    if (!wasActionTimerReset) {
                        actionTimer.resetTimer();
                        wasActionTimerReset = true;
                    } else {
                        if (actionTimer.getElapsedTimeSeconds() > preWaitTime) {
                            s_states = SHOOTING.BACKSPIN;
                            wasActionTimerReset = false;
                        }
                    }
                    break;
                case BACKSPIN:
                    Intake.setPower(0);
                    if (!wasActionTimerReset) {
                        actionTimer.resetTimer();
                        wasActionTimerReset = true;
                    } else {
                        if (actionTimer.getElapsedTimeSeconds() > 0.05) {
                            Intake.setPower(-0.2);
                        }

                        if (actionTimer.getElapsedTimeSeconds() > 0.2) {
                            Intake.setPower(0);
                            s_states = SHOOTING.LIFT_GATE;
                            wasActionTimerReset = false;
                        }
                    }
                    break;
                case LIFT_GATE:
                    Gate.setTargetPosition(0);

                    if (Gate.getCurrentPosition() <= 5) {
                        s_states = SHOOTING.INTAKE_FEED;
                    }
                    break;
                case INTAKE_FEED:
                    if (!wasActionTimerReset) {
                        actionTimer.resetTimer();
                        wasActionTimerReset = true;
                        Intake.setPower(shoot_trigger_intake_pwr * power_adj);
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 1) {
                        Intake.setPower(0);
                        wasActionTimerReset = false;
                        s_states = SHOOTING.CLOSE_GATE;
                    }
                    break;
                case CLOSE_GATE:
                    Gate.setTargetPosition(480);
                    if (Gate.getCurrentPosition() >= 475) {
                        s_states = SHOOTING.FOLLOW;
                    }
                    break;
                case FOLLOW:
                    follower.followPath(nextPath, pathMaxPower, true); // Path has temporal callback to reset the motor outake to intake power
                    setPathState(nextPathState, pathTimer);
                    break;
            }
        }
    }

    public void setPathState(int pState, Timer pathTimer) {
        pathState = pState;
        pathTimer.resetTimer();
        wasActionTimerReset = false;
    }
    // Old power and time based auto functions
    public void auto_red_go_pick_up_balls_from_near_shot_spot(
            int rot_time, int strafe_time,int ball_intake_time,
            int strafe_back_time, int rot_back_time, double power_adj,
            DcMotor FrontLeft, DcMotor FrontRight, DcMotor BackLeft, DcMotor BackRight,
            DcMotor Intake, DcMotor Gate) {
        // rotate counter clockwise for 45 degrees
        rotate_clockwise(FrontLeft, FrontRight, BackLeft, BackRight, wheel_pwr*power_adj);
        sleep(rot_time);
        stop_drive(FrontLeft, FrontRight, BackLeft, BackRight, 0);
        strafe_right(FrontLeft, FrontRight, BackLeft, BackRight, wheel_pwr*power_adj);
        sleep(strafe_time);
        stop_drive(FrontLeft, FrontRight, BackLeft, BackRight, 0);

        // pick up 2 balls from spike
        Intake.setPower(ball_pickup_intake_pwr*power_adj);
        move_forward(FrontLeft, FrontRight, BackLeft, BackRight, wheel_pwr*power_adj * ball_intake_move_power_adj);
        sleep(ball_intake_time);
        stop_drive(FrontLeft, FrontRight, BackLeft, BackRight, 0); // bot slips forward after this command
        sleep(100); // Add'l intake time needed to suck in ball TODO: can it be 250?
        Intake.setPower(0);

        // back off
        move_back(FrontLeft, FrontRight, BackLeft, BackRight, wheel_pwr*power_adj * ball_intake_move_power_adj);
        sleep((long) Math.round(ball_intake_time * 0.95)); // TODO: Do we need to add 50?
        stop_drive(FrontLeft, FrontRight, BackLeft, BackRight, 100); // bot slips forward after this command

        // strafe to near shot area
        strafe_left(FrontLeft, FrontRight, BackLeft, BackRight, wheel_pwr*power_adj);
        sleep((long) Math.round(strafe_back_time)); // - Math.min(100, (long) Math.round(strafe_time*0.1))));
        stop_drive(FrontLeft, FrontRight, BackLeft, BackRight, 0);

        // rotate to align with goal
        rotate_counter_clockwise(FrontLeft, FrontRight, BackLeft, BackRight, wheel_pwr * power_adj);
        sleep((long) Math.round(rot_back_time)); // 135ms
        stop_drive(FrontLeft, FrontRight, BackLeft, BackRight, 0);
    }

    public void auto_blue_go_pick_up_balls_from_near_shot_spot(
            int rot_time, int strafe_time,int ball_intake_time,
            int strafe_back_time, int rot_back_time, double power_adj,
            DcMotor FrontLeft, DcMotor FrontRight, DcMotor BackLeft, DcMotor BackRight,
            DcMotor Intake, DcMotor Gate) {
        // rotate counter clockwise for 45 degrees
        rotate_counter_clockwise(FrontLeft, FrontRight, BackLeft, BackRight, wheel_pwr*power_adj);
        sleep(rot_time);
        stop_drive(FrontLeft, FrontRight, BackLeft, BackRight, 0);
        strafe_left(FrontLeft, FrontRight, BackLeft, BackRight, wheel_pwr*power_adj);
        sleep(strafe_time);
        stop_drive(FrontLeft, FrontRight, BackLeft, BackRight, 0);

        // pick up 2 balls from spike
        Intake.setPower(ball_pickup_intake_pwr*power_adj);
        move_forward(FrontLeft, FrontRight, BackLeft, BackRight, wheel_pwr*power_adj * ball_intake_move_power_adj);
        sleep(ball_intake_time);
        stop_drive(FrontLeft, FrontRight, BackLeft, BackRight, 0); // bot slips forward after this command
        sleep(100); // Add'l intake time needed to suck in ball TODO: can it be 250?
        Intake.setPower(0);

        // back off
        move_back(FrontLeft, FrontRight, BackLeft, BackRight, wheel_pwr*power_adj * ball_intake_move_power_adj);
        sleep((long) Math.round(ball_intake_time * 0.95)); // TODO: Do we need to add 50?
        stop_drive(FrontLeft, FrontRight, BackLeft, BackRight, 100); // bot slips forward after this command

        // strafe to near shot area
        strafe_right(FrontLeft, FrontRight, BackLeft, BackRight, wheel_pwr*power_adj);
        sleep((long) Math.round(strafe_back_time)); // - Math.min(100, (long) Math.round(strafe_time*0.1))));
        stop_drive(FrontLeft, FrontRight, BackLeft, BackRight, 0);

        // rotate to align with goal
        rotate_clockwise(FrontLeft, FrontRight, BackLeft, BackRight, wheel_pwr * power_adj);
        sleep((long) Math.round(rot_back_time)); // 135ms
        stop_drive(FrontLeft, FrontRight, BackLeft, BackRight, 0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }
}

