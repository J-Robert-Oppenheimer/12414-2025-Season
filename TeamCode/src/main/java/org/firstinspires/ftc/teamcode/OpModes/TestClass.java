package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
@TeleOp(name = "Test Class")
public class TestClass extends LinearOpMode {
    //public CRServo testServo;

Servo testServo;
    GeneralHardwareMap map = new GeneralHardwareMap(this);


public int pos = 0;

//    @Override
    public void runOpMode() throws InterruptedException {
        //gHMap.initRANDOMOTOR("slideAngle");
        //gHMap.initRANDOMOTOR2("slideLength");
        map.init("Kineses");
        //testServo = hardwareMap.get(CRServo.class, "testServo");
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.square) {
                map.ex.setPosition(0);
                map.ex2.setPosition(0);
            }
            else if (gamepad1.triangle){
                map.ex.setPosition(0.5);
                map.ex2.setPosition(0.5);
            }
        }
    }
}




