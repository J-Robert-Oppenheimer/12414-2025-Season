package org.firstinspires.ftc.teamcode.OpModes.Auto;



import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.lang.annotation.Annotation;


public class SP {

    public static double widthOffset = 7.5;
    public static double lengthOffset = 8.5;
    public static double placementRange = lengthOffset + 1;
    public static double intakeRange = lengthOffset + 1;
    public static double netPlacementRange = lengthOffset + 3;
    public static double specPickupRange = lengthOffset + 7;

    public static double IntakeRangex(double theta) {
        return intakeRange * Math.cos(Math.toRadians(theta));
    }

    public static double IntakeRangey(double theta) {
        return intakeRange * Math.sin(Math.toRadians(theta));
    }

    public Vector IntakeRangeV(double theta, Vector orig) {
        return new Vector(orig.getXComponent() + (intakeRange * Math.cos(Math.toRadians(theta))), orig.getYComponent() + (intakeRange * Math.sin(Math.toRadians(theta))));
    }

    public static double netPlacementRangex(double theta) {
        return netPlacementRange * Math.cos(Math.toRadians(theta));
    }

    public static double netPlacementRangey(double theta) {
        return netPlacementRange * Math.sin(Math.toRadians(theta));
    }

    public Vector netPlacementRangeV(double theta, Vector orig) {
        return new Vector(orig.getXComponent() - (netPlacementRange * Math.cos(Math.toRadians(theta))), orig.getYComponent() - (netPlacementRange * Math.sin(Math.toRadians(theta))));
    }

    public static final Pose ObsZoneStart = new Pose(144 - lengthOffset, 96 - widthOffset, Math.toRadians(180));  // Obs start

    public static final Pose NetZoneStart = new Pose(144 - widthOffset, 48 + lengthOffset, Math.toRadians(180)); // Net start


    public static final Pose Specimen0 = new Pose(48 + lengthOffset, 67.5 + placementRange, Math.toRadians(180));

    public static final Pose Specimen1 = new Pose(48 + lengthOffset, 66 + placementRange, Math.toRadians(180)); // Specimen1

    public static final Pose Specimen2 = new Pose(48 + lengthOffset, 64.5 + placementRange, Math.toRadians(180)); // Specimen2

    public static final Pose Specimen3 = new Pose(48 + lengthOffset, 63 + placementRange, Math.toRadians(180)); // Specimen3

    public static final Pose Specimen4 = new Pose(48 + lengthOffset, 61.5 + placementRange, Math.toRadians(180)); // Specimen4

    public static final Pose Specimen5 = new Pose(48 + lengthOffset, 60, Math.toRadians(180)); // Specimen5

    public static final Pose NetSpecimen = new Pose(96, 82.5 + placementRange, Math.toRadians(180)); // Net Specimen


    public static final Pose NetDiagonal = new Pose(72 - netPlacementRangex(-45), 0 + netPlacementRangey(-45), Math.toRadians(-45)); // Net Diagonal

    public static final Pose Net = new Pose(72, 0, Math.toRadians(-45)); // Net


    public static final Pose SpikeWallObs = new Pose(98, 142, Math.toRadians(180)); // Obs Wall

    public static final Pose SpikeMidObs = new Pose(98, 132, Math.toRadians(180)); // Obs Mid

    public static final Pose SpikeSubObs = new Pose(98, 121, Math.toRadians(180)); // Obs Sub


    public static final Pose  SpikeWallNet = new Pose(98, 2, Math.toRadians(180)); // Net Wall

    public static final Pose SpikeMidNet = new Pose(98, 12, Math.toRadians(180)); // Net Mid

    public static final Pose SpikeSubNet = new Pose(98, 23, Math.toRadians(180)); // Net Sub


    public static final Pose SampleIntakeObsSub = new Pose(98 + intakeRange, 121, Math.toRadians(180)); // Obs Sub Intake

    public static final Pose SampleIntakeObsMid = new Pose(98 + intakeRange, 132, Math.toRadians(180)); // Obs Mid Intake

    public static final Pose SampleIntakeObsWall = new Pose(IntakeRangex(135) + 98, 142 - IntakeRangey(240), Math.toRadians(135)); // Obs Wall Intake

    public static final Pose SampleIntakeNetSub = new Pose(98 + intakeRange, 23, Math.toRadians(180)); // Net Sub Intake

    public static final Pose SampleIntakeNetMid = new Pose(98 + intakeRange, 12, Math.toRadians(180)); // Net Mid Intake

    public static final Pose SampleIntakeNetWall = new Pose(98 + IntakeRangex(-135), 2 + IntakeRangey(-135), Math.toRadians(-135)); // Net Wall Intake


    public static final Pose SampleDropSub = new Pose(138, 121 - placementRange, Math.toRadians(180)); // Sub Drop

    public static final Pose SampleDropMid = new Pose(138, 132 - placementRange, Math.toRadians(180)); // Mid Drop

    public static final Pose SampleDropWall = new Pose(138, 132 - placementRange, Math.toRadians(180)); // Wall Drop


    public static final Pose SamplePickupNet = new Pose(84, 47 - intakeRange, Math.toRadians(90)); // Net Pickup

    public static final Pose SamplePickupObs = new Pose(84, 97 + intakeRange, Math.toRadians(-90)); // Obs Pickup

    public static final Pose ObsZone = new Pose(138 - placementRange, 132, Math.toRadians(180)); // Obs Zone

    public static final Pose ObsZoneIntake = new Pose(144 - intakeRange, 120, Math.toRadians(180)); // Obs Intake

    public static final Pose ObsPark = new Pose(85, 98, Math.toRadians(-90));

    public static final Pose NetPark = new Pose(85, 46, Math.toRadians(90));


//
//
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
}
