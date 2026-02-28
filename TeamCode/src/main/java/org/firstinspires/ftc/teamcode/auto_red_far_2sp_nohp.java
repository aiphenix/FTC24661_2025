package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class auto_red_far_2sp_nohp extends OpMode { // no human player
    private Follower follower;
    private Timer pathTimer, actionTimer;
    private boolean wasActionTimerReset = false;
    private int pathState;
    ftc_2025_functions ftc_fns = new ftc_2025_functions();

    DcMotorEx ShootLeft;
    DcMotorEx ShootRight;
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;
    DcMotor Gate;
    DcMotor Intake;
    Servo HoodLeft;
    Servo HoodRight;
    double power_adj;


    // PedroPath points
    private final Pose start = new Pose(144 - 57.33, 8.937, Math.toRadians(180 - 90)); // Start pose of our robot
    private final Pose score = new Pose(144 - 59.156, 20.957, Math.toRadians(180 - 115)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle
    private final Pose intake1CP = new Pose(144 - 61.721, 35.292); // Control Point
    private final Pose intake1End = new Pose(144 - 13.41, 40, Math.toRadians(180 - 182)); // Highest (First Set) of Artifacts from the Spike Mark
    private final Pose intake2CP = new Pose(144 - 61.113, 73.57); // Control Point
    private final Pose intake2End = new Pose(144 - 14.59, 58, Math.toRadians(180 - 182)); // Lowest (Third Set) of Artifacts from the Spike Mark
    private final Pose park = new Pose(144 - 30.153, 82.491, Math.toRadians(180 - 180)); // Park in front of gate at end of auto
    private PathChain startToScore, scoreToIntake1, intake1ToScore, scoreToIntake2,
            intake2ToScore, scoreToPark;

    private enum SHOOTING {
        PREWAIT,
        BACKSPIN,
        LIFT_GATE,
        INTAKE_FEED,
        CLOSE_GATE,
        FOLLOW
    }

    SHOOTING s_states = SHOOTING.PREWAIT;

    public void buildPaths(double power_adj) {
        // Intake enable and shooter rev is done using temporal callbacks
        startToScore = follower.pathBuilder()
                .addPath(new BezierLine(start, score))
                .setLinearHeadingInterpolation(start.getHeading(), score.getHeading(), 0.75)
                .addParametricCallback(0, () -> ftc_fns.zero_gate(Gate))
                .build();

        // Intake and score first artifact group
        scoreToIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(score, intake1CP, intake1End))
                .setLinearHeadingInterpolation(score.getHeading(), intake1End.getHeading(), 0.5)
                .addParametricCallback(0.2, () -> Intake.setPower(ftc_fns.ball_pickup_intake_pwr * power_adj))
                .build();
        intake1ToScore = follower.pathBuilder()
                .addPath(new BezierLine(intake1End, score))
                .setLinearHeadingInterpolation(intake1End.getHeading(), score.getHeading(), 0.99)
                .addParametricCallback(0.99, () -> Intake.setPower(0))
                .build();

        // Intake and score second artifact group
        scoreToIntake2 = follower.pathBuilder()
                .addPath(new BezierCurve(score, intake2CP, intake2End))
                .setLinearHeadingInterpolation(score.getHeading(), intake2End.getHeading(), 0.7)
                .addParametricCallback(0.3, () -> Intake.setPower(ftc_fns.ball_pickup_intake_pwr * power_adj))
                .build();
        intake2ToScore = follower.pathBuilder()
                .addPath(new BezierLine(intake2End, score))
                .setLinearHeadingInterpolation(intake2End.getHeading(), score.getHeading(), 0.99)
                .addParametricCallback(0.99, () -> Intake.setPower(0))
                .build();

        // Park in front of gate in prep for teleop. Gets robot off launch line for auto leave
        scoreToPark = follower.pathBuilder()
                .addPath(new BezierLine(score, park))
                .setLinearHeadingInterpolation(score.getHeading(), park.getHeading())
                .addParametricCallback(0.1, () -> ftc_fns.set_shooter_speed(
                        0, false, ShootLeft, ShootRight, telemetry, gamepad1))
                .addParametricCallback(0.05, () -> Gate.setTargetPosition(0))
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                waitThenFollowPath(1, startToScore,  1, 1);
                break;
            case 1:
                shootThenFollowPath(1, scoreToIntake1, 1, 2);
                break;
            case 2:
                waitThenFollowPath(0.1, intake1ToScore, 1, 3);
                break;
            case 3:
                shootThenFollowPath(0, scoreToIntake2, 1, 4);
                break;
            case 4:
                waitThenFollowPath(0.1, intake2ToScore, 1, 5);
                break;
            case 5:
                shootThenFollowPath(0, scoreToPark, 1, -1);
                break;
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/

    public void shootThenFollowPath (double preWaitTime, PathChain nextPath, double pathMaxPower, int nextPathState) {
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
                        if (actionTimer.getElapsedTimeSeconds() > 0.02) {
                            Intake.setPower(-0.2);
                        }

                        if (actionTimer.getElapsedTimeSeconds() > 0.1) {
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
                        Intake.setPower(ftc_fns.shoot_trigger_intake_pwr * power_adj);
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
                    setPathState(nextPathState);
                    break;
            }
        }
    }

    public void waitThenFollowPath (double waitTime, PathChain nextPath, double pathMaxPower, int nextPathState) {
        if(!follower.isBusy()) {
            if (!wasActionTimerReset) {
                actionTimer.resetTimer();
                wasActionTimerReset = true;
            }

            if (actionTimer.getElapsedTimeSeconds() > waitTime) {
                follower.followPath(nextPath, pathMaxPower, true);
                s_states = SHOOTING.PREWAIT;
                wasActionTimerReset = false;
                setPathState(nextPathState);
            }
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        wasActionTimerReset = false;
    }

//    public void setShootState()

    @Override
    public void init() {
        // Define Devices
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Gate = hardwareMap.get(DcMotor.class, "Gate");
        HoodLeft = hardwareMap.get(Servo.class, "HoodLeft");
        HoodRight = hardwareMap.get(Servo.class, "HoodRight");

        ShootLeft = hardwareMap.get(DcMotorEx.class, "ShootLeft");
        ShootRight = hardwareMap.get(DcMotorEx.class, "ShootRight");

        VoltageSensor ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        ftc_fns.set_motor_orientations_PIDF_and_zero_power_behavior(
                FrontLeft, FrontRight, BackLeft, BackRight,
                ShootLeft, ShootRight, Intake, Gate, HoodLeft, HoodRight
        );

        // adjust power based on voltage
        double curr_vol = ControlHub_VoltageSensor.getVoltage();
        power_adj = ftc_fns.adjust_power(curr_vol, telemetry);

        pathTimer = new Timer();
        actionTimer = new Timer();
        wasActionTimerReset = false;
        follower = Constants.createFollower(hardwareMap);
        buildPaths(power_adj);
        follower.setStartingPose(start);
        wasActionTimerReset = false;
    }

    @Override
    public void start() {
        // Run shooter wheel during entire auto session
        ftc_fns.set_shooter_speed(
                ftc_fns.far_shot_shooter_rpm_for_auto, false, ShootLeft,
                ShootRight, telemetry, gamepad1);
        HoodLeft.setPosition(ftc_fns.far_shot_hood_servo_pos);
        HoodRight.setPosition(ftc_fns.far_shot_hood_servo_pos);
//        Gate.setPower(ftc_fns.init_gate_lift_pwr * power_adj); // zero gate happens at end of path startToScore
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower State", follower.isBusy());
        telemetry.addData("SHOOTING STATE: ", s_states.toString());
        telemetry.addData("Is Timer Set: ", wasActionTimerReset);
        telemetry.addData("Action Timer", actionTimer.getElapsedTimeSeconds());
        telemetry.addData("Gate Position", Gate.getCurrentPosition());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }
}