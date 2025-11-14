package org.firstinspires.ftc.teamcode.vittoz.p04_lanceur;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Classe pour gerer le lanceur des artefacts
 */
public class Lanceur {

    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private DcMotor Moteur_G;
    private DcMotor Moteur_D;


    public Lanceur(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        this.Moteur_D = hardwareMap.get(DcMotor.class, "Moteur_lanceur_D");
        this.Moteur_G = hardwareMap.get(DcMotor.class, "Moteur_lanceur_G");
    }

    public void lancer(float puiisance)
    {
        Moteur_D.setPower(-puiisance);
        Moteur_G.setPower(puiisance);
    }

}
