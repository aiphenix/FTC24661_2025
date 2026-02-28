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
public class auto_red_far_leave extends LinearOpMode {
    ftc_2025_functions ftc_fns = new ftc_2025_functions();

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
        limelight.pipelineSwitch(6);
        limelight.start();
        telemetry.setMsTransmissionInterval(10);

        // adjust power based on voltage
        double curr_vol = ControlHub_VoltageSensor.getVoltage();
        double power_adj = ftc_fns.adjust_power(curr_vol, telemetry);

        waitForStart();

        if (opModeIsActive()) {
            Gate.setPower(ftc_fns.init_gate_lift_pwr * power_adj); // Initial gate lift with low power
            sleep(1500);
            ftc_fns.zero_gate(Gate);

            // leave shooting area
            ftc_fns.strafe_right(FrontLeft, FrontRight, BackLeft, BackRight, ftc_fns.wheel_pwr * power_adj);
            sleep(ftc_fns.auton_time_to_leave_near_shot_area);
            ftc_fns.stop_drive(FrontLeft, FrontRight, BackLeft, BackRight,0);
        }
    }
}