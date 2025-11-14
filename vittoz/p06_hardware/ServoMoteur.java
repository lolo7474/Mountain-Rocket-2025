package org.firstinspires.ftc.teamcode.vittoz.p06_hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Classe pour gerer un servo moteur
 */
public class ServoMoteur {
    public final Telemetry telemetry;
    public final HardwareMap hardwareMap;
    public Servo servo;
    public double possitionServoInitial;
    double possitionServo;

    public ServoMoteur(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    /**
     * Initialiser le servo a sa position initiale
     */
    public void initServo() {
        servo.setPosition(possitionServoInitial);
    }

    /**
     * Definir la position du servo
     * @param position Position entre 0 et 1
     */
    public void setPosition(double position)
    {
        possitionServo = position;
        servo.setPosition(possitionServo);
        telemetry.addData("Servo position", position);
    }
}
