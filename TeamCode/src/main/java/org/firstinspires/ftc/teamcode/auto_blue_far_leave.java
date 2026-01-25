// This entire codes works at 12.8V battery power
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Autonomous
public class auto_blue_far_leave extends LinearOpMode {
    ftc_2025_functions ftc_fns = new ftc_2025_functions(hardwareMap);

    @Override
    public void runOpMode() {
        // Define Devices
        DcMotorEx FrontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        DcMotorEx FrontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        DcMotor BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        DcMotor BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        DcMotorEx ShootLeft = hardwareMap.get(DcMotorEx.class, "ShootLeft");
        DcMotorEx ShootRight = hardwareMap.get(DcMotorEx.class, "ShootRight");

        DcMotor Intake = hardwareMap.get(DcMotor.class, "Intake");
        DcMotor Gate = hardwareMap.get(DcMotor.class, "Gate");
        Servo HoodLeft = hardwareMap.get(Servo.class, "HoodLeft");
        Servo HoodRight = hardwareMap.get(Servo.class, "HoodRight");

        VoltageSensor ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        ftc_fns.set_motor_orientations_PIDF_and_zero_power_behavior(
                FrontLeft, FrontRight, BackLeft, BackRight,
                ShootLeft, ShootRight, Intake, Gate, HoodLeft, HoodRight
        );

        // Set up limelight
        // Motors and Accessories
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        limelight.start();
        telemetry.setMsTransmissionInterval(10);

        // adjust power based on voltage
        double curr_vol = ControlHub_VoltageSensor.getVoltage();
        double power_adj = ftc_fns.adjust_power(curr_vol, telemetry);

        waitForStart();

        if (opModeIsActive()) {
            // Run shooter wheel during entire auto session
//            ftc_fns.set_shooter_speed(
//                    ftc_fns.near_shot_shooter_rpm, false, ShootLeft, ShootRight);
//            HoodLeft.setPosition(ftc_fns.near_shot_hood_servo_pos);
//            HoodRight.setPosition(ftc_fns.near_shot_hood_servo_pos);

            // Backoff bot from initial pos
//            ftc_fns.move_back(FrontLeft, FrontRight, BackLeft, BackRight,
//                    ftc_fns.auton_init_move_back_pwr * power_adj);
            Gate.setPower(ftc_fns.init_gate_lift_pwr * power_adj); // Initial gate lift with low power
//            sleep(ftc_fns.auton_init_move_back_time);
//            ftc_fns.stop_drive(FrontLeft, FrontRight, BackLeft, BackRight,0);
//            ftc_2025_functions.AimResultPair result_pair = ftc_fns.auto_aim(
//                    true, FrontLeft, FrontRight, BackLeft, BackRight,
//                    limelight, telemetry, gamepad1);
//            boolean aimed = result_pair.isSuccess();
//            long aim_time_elapsed = result_pair.getTimeElapsed();
//            telemetry.addData("Auto Aim Took", aim_time_elapsed);
//            long additional_sleep_needed = (ftc_fns.total_init_gate_lift_time
//                    - ftc_fns.auton_init_move_back_time - aim_time_elapsed);
//            if (additional_sleep_needed > 0) {
//                sleep(additional_sleep_needed);
//            }
            sleep(3500);
            ftc_fns.zero_gate(Gate);

            // Preloaded shot0
//            ftc_fns.make_near_shot(power_adj, true, Intake, Gate);

//            // shot1
//            ftc_fns.auto_blue_go_pick_up_balls_from_near_shot_spot(
//                    ftc_fns.rot_time1, ftc_fns.strafe_time1, ftc_fns.intake_time1,
//                    ftc_fns.strafe_back_time1, ftc_fns.rot_back_time1, power_adj,
//                    FrontLeft, FrontRight, BackLeft, BackRight, Intake, Gate);
//            ftc_fns.auto_aim(
//                    true, FrontLeft, FrontRight, BackLeft, BackRight,
//                    limelight, telemetry, gamepad1);
//            ftc_fns.lift_gate(true, Gate, Intake);
//            ftc_fns.make_near_shot(power_adj, true, Intake, Gate);
//
//            // shot2
//            ftc_fns.auto_blue_go_pick_up_balls_from_near_shot_spot(
//                    ftc_fns.rot_time2, ftc_fns.strafe_time2, ftc_fns.intake_time2,
//                    ftc_fns.strafe_back_time2, ftc_fns.rot_back_time2, power_adj,
//                    FrontLeft, FrontRight, BackLeft, BackRight, Intake, Gate);
//            ftc_fns.auto_aim(
//                    true, FrontLeft, FrontRight, BackLeft, BackRight,
//                    limelight, telemetry, gamepad1);
//            ftc_fns.lift_gate(true, Gate, Intake);
//            ftc_fns.make_near_shot(power_adj, true, Intake, Gate);

//            // shot3
//            ftc_fns.auto_blue_go_pick_up_balls_from_near_shot_spot(
//                    ftc_fns.rot_time3, ftc_fns.strafe_time3, ftc_fns.intake_time3,
//                    ftc_fns.strafe_back_time3, ftc_fns.rot_back_time3, power_adj,
//                    FrontLeft, FrontRight, BackLeft, BackRight, Intake, Gate);
//            ftc_fns.auto_aim(
//                    true, FrontLeft, FrontRight, BackLeft, BackRight,
//                    limelight, telemetry, gamepad1);
//            ftc_fns.lift_gate(true, Gate, Intake);
//            ftc_fns.make_near_shot(power_adj, false, Intake, Gate);

            // leave shooting area
            ftc_fns.strafe_left(FrontLeft, FrontRight, BackLeft, BackRight, ftc_fns.wheel_pwr * power_adj);
            sleep(ftc_fns.auton_time_to_leave_near_shot_area);
            ftc_fns.stop_drive(FrontLeft, FrontRight, BackLeft, BackRight,0);

//            ftc_fns.power_down_shooter(ShootLeft, ShootRight);
        }
    }
}