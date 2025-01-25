package org.firstinspires.ftc.teamcode.OpModes.Auto;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.GeneralHardwareMap;

import java.lang.annotation.Annotation;

@Config
@Autonomous (name = "PedroPascal", group = "Autonomous")
public class PedroPascal extends OpMode {
    GeneralHardwareMap gHMap = new GeneralHardwareMap(this);
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private double LowJointPos;
    private double HighJointPos;


    private PathChain obs2Spec5, spec2Park;

    public void buildPaths() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        obs2Spec5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SP.ObsZoneStart), new Point(SP.Specimen5)))
                .setLinearHeadingInterpolation(SP.ObsZoneStart.getHeading(), SP.Specimen5.getHeading())
                .build();

        spec2Park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SP.Specimen5), new Point(SP.ObsPark)))
                .setLinearHeadingInterpolation(SP.Specimen5.getHeading(), SP.ObsPark.getHeading())

                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
                follower.followPath(obs2Spec5);
//                gHMap.smoothJoints(LowJointPos,0.41,0.34);
//                LowJointPos = 0.41;
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()){
                    follower.followPath(spec2Park);
                }
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
//        gHMap.init("Kineses");
//        gHMap.smoothJoints(0,0.28,0.30);
        LowJointPos = 0.28;

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(SP.ObsZoneStart);
        buildPaths();
    }

//    @Override
//    public String name() {
//        return "";
//    }
//
//    @Override
//    public String group() {
//        return "";
//    }
//
//    @Override
//    public String preselectTeleOp() {
//        return "";
//    }
//
//    @Override
//    public Class<? extends Annotation> annotationType() {
//        return null;
//    }


    //    public void init() {
//        follower.followPath(one_zeroObs);
//        buildPaths();
//    }
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }

//        follower.telemetryDebug(telemetryA);




}
