package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class Limelight {
    private final Limelight3A limelight;
    private LLResult result;
    private int tagId;
    private double dist, xFromTag, yFromTag, aFromTag;

    public Limelight(HardwareMap hardwareMap, int pipeLine) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeLine);
        limelight.setPollRateHz(50);
        limelight.start();

        update();
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public void update() {
        result = limelight.getLatestResult();
        if (result != null && result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            LLResultTypes.FiducialResult fr = fiducialResults.get(0); // first tag seen
            tagId = fr.getFiducialId();
            dist = result.getBotposeAvgDist();
            xFromTag = result.getTx();
            yFromTag = result.getTy();
            aFromTag = result.getTa();
        } else {
            tagId = -1; // no tag seen
            dist = -999;   // or some safe default
            xFromTag = -999;
            yFromTag = -999;
            aFromTag = -999;
        }
    }

    public int getTagID() {
        update();
        return tagId;
    }

    public double getDistance() {
        update();
        return dist;
    }

    public double getxFromTag() {
        update();
        return xFromTag;
    }

    public double getyFromTag() {
        update();
        return yFromTag;
    }

    public double getaFromTag() {
        update();
        return aFromTag;
    }
}