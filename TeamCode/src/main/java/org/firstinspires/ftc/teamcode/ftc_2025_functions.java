// This entire codes if at 13.7V batter
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class ftc_2025_functions extends LinearOpMode {
    // --------------- Constants --------------
    public double pidf_p = 175;
    public double pidf_f = 16.8;

    // Initialization
    public double init_gate_lift_pwr = -0.2;
    public int gate_down_position  = 490;

    // Shooting
    public double near_shot_hood_servo_pos = 0.77;
    public double near_shot_hood_servo_pos_for_auto = near_shot_hood_servo_pos + 0.1;
    public double far_shot_hood_servo_pos = 0.71;
    public double near_shot_shooter_rpm = 2865; // 3050;
    public double near_shot_shooter_rpm_for_auto = near_shot_shooter_rpm;
    public double far_shot_shooter_rpm = 3750; // 4050;
    public double shoot_trigger_intake_pwr = 1;

    // Movement
    public double wheel_pwr = 1;
    public double ball_pickup_intake_pwr = 0.7;
    public long auton_time_to_leave_near_shot_area = 650;

    // --------------- Functions --------------
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
            Servo HoodLeft, Servo HoodRight //, HardwareMap hardwareMap
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
        ShootLeft.setVelocityPIDFCoefficients(pidf_p, 0, 0, pidf_f);
        ShootRight.setVelocityPIDFCoefficients(pidf_p, 0, 0, pidf_f);

        // By default, the expansion hubs communicate over the slow I2C bus for every
        // getCurrentPosition() call. Turn on "Fast Mode" for how Control Hub talks to motors
        // and sensors.
        // BUT THIS DOESN'T WORK BECAUSE IT CAUSES LIMELIGHT TO MALFUNCTION
//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
    }

    // gate function
    public void lift_gate(boolean wait_till_completion, DcMotor Gate, DcMotor Intake) {
        // stop intake in case it is still forward spinning.
//        Intake.setPower(0);
//        sleep(25);
//        // unroll the intake to release pressure on gate
//        Intake.setPower(-0.2);
//        sleep(75);
//        Intake.setPower(0);

        // Lift gate
        Gate.setTargetPosition(0);

        if (wait_till_completion) {
            while (Gate.getCurrentPosition() > 175) {
                sleep(10);
            }
        }
    }
    public void close_gate(DcMotor Gate) {
        Gate.setTargetPosition(gate_down_position);
    }
    public void zero_gate(DcMotor Gate) {
        // backspin motor by a tiny bit to release pressure
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
        int max_tries = 3_000_000;
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
        // Display read counter - this refreshes way too fast and overwrites other messages
//        telemetry.addData("Total Limelight Readings", total_read_counter);
//        telemetry.update();
        if (!txs.isEmpty()) { // Has some readings, take average
            final_values.add(average(txs));
            final_values.add(average(tys));
            final_values.add(average(tas));
            telemetry.addData("Limelight X values:", txs);
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
        double tolerance_thresh = 0.2; // 20% tolerance
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

    public boolean is_near(Limelight limelight, Telemetry telemetry) {
        List<Double> bot_pos = get_lime_reading(limelight, telemetry,false);
        if (bot_pos.isEmpty()) {
            telemetry.addData("Near / Far check", "FAILED!");
            telemetry.update();
            return true; // return true because it doesn't matter as the bot can't aim anyway
        } else {
            telemetry.addData("Target Area %", bot_pos.get(2));
            telemetry.update();
            return bot_pos.get(2) > 0.6;
        }
    }

    /**
     * New Implementation of near/far check with Limelight.java class.
     * <p>
     * Returns
     *     -1 if cannot see tag
     *     1 if bot is in near zone (tag area > 0.8)
     *     0 if bot is in far zone (tag area <= 0.8)
     *
     * @param limelight Limelight object
     */
    public int is_near_1(Limelight limelight) {
        double area = limelight.getaFromTag();
        if (area < 0.01) {
            return -1;
        } else if (area > 0.6) {
            return 1;
        } else {
            return 0;
        }
    }

    public int is_near_2(Limelight limelight, Telemetry telemetry) {
        List<Double> bot_pos = get_lime_reading(limelight, telemetry,false);
        if (bot_pos.isEmpty()) {
            telemetry.addData("Near / Far check", "FAILED!");
            return -1; // return true because it doesn't matter as the bot can't aim anyway
        } else {
            telemetry.addData("Target Area %", bot_pos.get(2));
            if (bot_pos.get(2) > 0.6) {
                return 1;
            } else {
                return 0;
            }
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
        sleep(10); // This proves to be necessary for aim to work

        telemetry.addLine("Auto_aim started");
//        Instant start = Instant.now();
        double auto_aim_x_speed_booster = 1;
        double auto_aim_max_x_speed_booster = 1;
        double auto_aim_y_speed_booster = 1;
        double auto_aim_max_y_speed_booster = 1;
        int max_aim_steps = 40;
        int max_x_y_tries = 350;

        // Check if the shot is near or far4
        int near_shot = is_near_2(limelight, telemetry);
        // Set targets and power/time needed for incremental adjustment
        double target_x = 0;
        double x_tol = 3.5; // This value has to be above 3 because post aim inertia drift is ~1-2 degrees
        double x_power = 0.3;
        long x_drive_time = 1;

        double target_y = -11.6;
        double y_tol = 0.15;
        double y_power = 0.2;
        long y_drive_time = 10;

        if (near_shot == 0) { // far shooting
            if (is_blue) {
                target_x = -2;
            } else {
                target_x = 2;
            }
            x_tol = 2; // This value has to be above 3 because post aim inertia drift is ~1-2 degrees
            x_power = 0.2;
            x_drive_time = 1;

            target_y = 7.9;
            y_tol = 0.1;
            y_power = 0.2;
            y_drive_time = 10;
        }

        boolean aimed = false;
        double final_x = -99, final_y = -99;
        int fail_reason = 0;
        int aim_steps_taken = 0;
        long x_tries = 0;

        List<Double> limeReading;
        if (near_shot == -1) {
            fail_reason = 1;
            telemetry.addData("Cannot see April tag at all!", fail_reason);
        } else {
            telemetry.addLine("Entering Aiming");
            aimloop: while (!aimed) {
                telemetry.addData("Aiming step", aim_steps_taken);

                limeReading = get_lime_reading(limelight, telemetry,false);
                if (limeReading.isEmpty()) {
                    fail_reason = 2;
                    telemetry.addData("Failed to read initial X", fail_reason);
                    break;
                }
                double curr_x = limeReading.get(0);
                double curr_x_diff = curr_x - target_x;
                x_tries = 0;
                while (Math.abs(curr_x_diff) > x_tol) {
                    if (curr_x_diff < -x_tol) {
                        double power_boost = Math.min(
                                Math.max(auto_aim_x_speed_booster * (-curr_x_diff/x_tol), 1), auto_aim_max_x_speed_booster);
                        rotate_clockwise(FrontLeft, FrontRight, BackLeft, BackRight, x_power * power_boost);
                        sleep(x_drive_time);
                    } else {
                        double power_boost = Math.min(
                                Math.max(auto_aim_x_speed_booster * (curr_x_diff/x_tol), 1), auto_aim_max_x_speed_booster);
                        rotate_counter_clockwise(FrontLeft, FrontRight, BackLeft, BackRight, x_power * power_boost);
                        sleep(x_drive_time);
                    }

                    limeReading = get_lime_reading(limelight, telemetry, false);
                    if (limeReading.isEmpty()) {
                        fail_reason = 3;
                        telemetry.addData("Failed to read X while turning", fail_reason);
                        break aimloop;
                    }
                    curr_x = limeReading.get(0);
                    curr_x_diff = curr_x - target_x;
                    x_tries++;
                    if (x_tries > max_x_y_tries) {
                        fail_reason = 4;
                        telemetry.addData("Reached max X tries", fail_reason);
                        break aimloop;
                    }
                    if (gamepad1.backWasPressed()) {
                        break aimloop;
                    }
                }
                // Stop rotation given X is aimed
                stop_drive(FrontLeft, FrontRight, BackLeft, BackRight, 0);

                double curr_y;
                limeReading = get_lime_reading(limelight, telemetry, false);
                if (!skip_y) {
                    if (limeReading.isEmpty()) {
                        fail_reason = 5;
                        telemetry.addData("Failed to read initial Y", fail_reason);
                        break;
                    }
                    curr_y = limeReading.get(1);
                    double curr_y_diff = curr_y - target_y;
                    long y_tries = 0;

                    while (Math.abs(curr_y_diff) > y_tol) {
                        if (curr_y - target_y < -y_tol) {
                            double power_boost = Math.min(
                                    Math.max(auto_aim_y_speed_booster * (-curr_y_diff/y_tol), 1), auto_aim_max_y_speed_booster);
                            move_back(FrontLeft, FrontRight, BackLeft, BackRight,
                                    y_power * power_boost);

                            sleep((long) (y_drive_time/power_boost));
                        } else {
                            double power_boost = Math.min(
                                    Math.max(auto_aim_y_speed_booster * (curr_y_diff/y_tol), 1), auto_aim_max_y_speed_booster);
                            move_forward(FrontLeft, FrontRight, BackLeft, BackRight,
                                    y_power * power_boost);

                            sleep((long) (y_drive_time/power_boost));

                        }

                        limeReading = get_lime_reading(limelight, telemetry,false);
                        if (limeReading.isEmpty()) {
                            fail_reason = 6;
                            telemetry.addData("Failed to read Y while moving", fail_reason);
                            break aimloop;
                        }
                        curr_y = limeReading.get(1);
                        curr_y_diff = curr_y - target_y;
                        y_tries++;
                        if (y_tries > max_x_y_tries) {
                            fail_reason = 7;
                            telemetry.addData("Reached max Y tries", fail_reason);
                            break aimloop;
                        }
                        if (gamepad1.backWasPressed()) {
                            break aimloop;
                        }
                    }
                    // stop y-direction movements once aimed
                    stop_drive(FrontLeft, FrontRight, BackLeft, BackRight, 0);
                }

                // check if aimed
                limeReading = get_lime_reading(limelight, telemetry,false);
                if (limeReading.isEmpty()) {
                    fail_reason = 8;
                    telemetry.addData("Cannot read April Tag in final check", fail_reason);
                    break;
                }
                curr_x = limeReading.get(0);
                curr_y = limeReading.get(1);
                if (!skip_y && Math.abs(curr_x - target_x) <= x_tol && Math.abs(curr_y - target_y) <= y_tol) {
                    aimed = true;
                } else if (skip_y && Math.abs(curr_x - target_x) <= x_tol) {
                    aimed = true;
                }
                final_x = curr_x;
                final_y = curr_y;
                aim_steps_taken++;
                if (aim_steps_taken > max_aim_steps) {
                    fail_reason = 9;
                    telemetry.addData("Reached max aim steps", fail_reason);
                    break;
                }
                if (gamepad1.backWasPressed()) {
                    break;
                }
            }
            // If breaking out of aimloop, stop bot:
            stop_drive(FrontLeft, FrontRight, BackLeft, BackRight, 0);
        }

        // Sanity check
//        limeReading = get_lime_reading(limelight, telemetry,false);
//        double actual_x = -99, actual_y = -99;
//        if (!limeReading.isEmpty()) {
//            actual_x = limeReading.get(0);
//            actual_y = limeReading.get(1);
//        }

        if (!aimed) {
            telemetry.addData("Auto Aim [FINAL CHECK]", "Failed... ");
            telemetry.addData("Fail Reason", fail_reason);
        } else {
            telemetry.addData("Auto Aim [FINAL CHECK]", "Success!");
        }
        telemetry.addData("Aim Steps Taken", aim_steps_taken);
        telemetry.addData("X Tries", x_tries);

        telemetry.addData("Target X", target_x);
        telemetry.addData("X Tolerance", x_tol);
        telemetry.addData("Final X", final_x);
//        telemetry.addData("Actual X", actual_x);

        telemetry.addData("Target Y", target_y);
        telemetry.addData("Y Tolerance", y_tol);
        telemetry.addData("Final Y", final_y);
//        telemetry.addData("Actual Y", actual_y);

        telemetry.update();

        // The following Java timer call require API v26. We're at v24, hence not possible
//        Instant end = Instant.now();
//        Duration duration = Duration.between(start, end);
//        long time_elapsed = duration.toMillis();
        long time_elapsed = 300; // fake time
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
    public double convert_tps_to_rpm(double tps) {
        int ticks_per_round = 28;
        return tps / ticks_per_round * 60;
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
//        while (Math.abs(shooter_left_act_vel / tps - 1) > 0.01
//                && Math.abs(shooter_right_act_vel / tps - 1) > 0.01) {
        while (Math.abs((shooter_left_act_vel + shooter_right_act_vel) / (2 * tps) - 1) > 0.01) {
            sleep(50); // TODO: Can we tune down to 50?
            shooter_left_act_vel = ShootLeft.getVelocity();
            shooter_right_act_vel = ShootRight.getVelocity();
            telemetry.addData("Shooter want speed (rpm)", convert_tps_to_rpm(tps));
            telemetry.addData("Shooter left actual speed (rpm)", convert_tps_to_rpm(shooter_left_act_vel));
            telemetry.addData("Shooter right actual speed (rpm)", convert_tps_to_rpm(shooter_right_act_vel));
            telemetry.update();
            if (gamepad1.backWasPressed()) {
                break;
            }
        }
    }
    public void make_near_shot(double power_adj, boolean open_gate_before, boolean close_gate_after, DcMotor Intake, DcMotor Gate) {
        if (open_gate_before) {
            lift_gate(true, Gate, Intake);
        }
        // shoot
        Intake.setPower(shoot_trigger_intake_pwr * power_adj);
        sleep(900);
        Intake.setPower(0);

        // close gate
        if (close_gate_after) {
            sleep(60); // sleep to be safe no ball is being pushed by intake
            close_gate(Gate);
        }
    }
    public void make_far_shot(double power_adj, boolean open_gate_before, boolean close_gate_after, DcMotor Intake, DcMotor Gate) {
        if (open_gate_before) {
            lift_gate(true, Gate, Intake);
        }

        Intake.setPower(shoot_trigger_intake_pwr * power_adj);
        sleep(900);
        Intake.setPower(0);

        // close gate
        if (close_gate_after) {
            sleep(60); // sleep to be safe no ball is being pushed by intake
            close_gate(Gate);
        }
    }
    public void power_down_shooter(DcMotorEx ShootLeft, DcMotorEx ShootRight) {
        ShootLeft.setPower(0);
        ShootRight.setPower(0);
    }

    /**
     * Get distance by limelight and guard against invalid readings.
     *
     * @param limelight Limelight class object
     * @param near_shot Boolean on whether you're attempting a near shot
     * @return a double value that is the distance from target in meters
     */
    public double get_dist_safe(Limelight limelight, Boolean near_shot) {
        double near_default_value = 1;
        double far_default_value = 3;
        int max_n_readings = 10;
        double min_valid_dist = 0.5;

        double dist = 0;
        int n_readings = 0;
        while (dist < min_valid_dist) {
            dist = limelight.getDistance();
            n_readings++;
            if (n_readings > max_n_readings) {
                break;
            }
        }
        if (dist < min_valid_dist) {
            if (near_shot) {
                dist = near_default_value;
            } else {
                dist = far_default_value;
            }
        }
        return dist;
    }


    @Override
    public void runOpMode() throws InterruptedException {    }
}

