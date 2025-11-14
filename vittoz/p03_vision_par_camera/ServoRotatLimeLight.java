package org.firstinspires.ftc.teamcode.vittoz.p03_vision_par_camera;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vittoz.p06_hardware.ServoMoteur;

/**
 * Classe pour gerer le servo de rotation de la limelight
 */
public class ServoRotatLimeLight extends ServoMoteur {

    /**
     * Constructeur: Classe pour gerer le servo de rotation de la limelight
     * @param telemetry
     * @param hardwareMap
     */
    public ServoRotatLimeLight(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);

        servo = hardwareMap.get(Servo.class, "servo_rotat_limelight");
        possitionServoInitial = 0.5;

        initServo();
        telemetry.addData("Servo rotation Limelight initialisation", "OK");

    }

    /**
     * Faire suivre la limelight en fonction du bearing
     * @param bearing Bearing de la cible
     * @param bearingMax Bearing max de la camera
     */
    public void folow(double bearing, double bearingMax) {
        double pas = 0.0005 * (Math.abs(bearing)/bearingMax);
        if (bearing > 0.5) {
            setPosition(servo.getPosition() + pas);
        } else if (bearing < -0.5) {
            setPosition(servo.getPosition() - pas);
        }

    }
}
