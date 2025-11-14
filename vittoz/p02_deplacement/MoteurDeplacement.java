package org.firstinspires.ftc.teamcode.vittoz.p02_deplacement;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Classe pour gerer les moteurs de deplacement du robot
 */
public class MoteurDeplacement {
    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private DcMotor Moteur_AVG;
    private DcMotor Moteur_ARG;
    private DcMotor Moteur_AVD;
    private DcMotor Moteur_ARD;

    double powerAVG;
    double powerARG;
    double powerAVD;
    double powerARD;

    boolean premiereSetPower = true;

    double multiplicateurPuissance = 1;

    /**
     * Constructeur: Classe pour gerer les moteurs de deplacement du robot
     * @param telemetry
     * @param hardwareMap
     */
    public MoteurDeplacement(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        Moteur_AVG = hardwareMap.get(DcMotor.class, "Moteur_AVG");
        Moteur_ARG = hardwareMap.get(DcMotor.class, "Moteur_ARG");
        Moteur_AVD = hardwareMap.get(DcMotor.class, "Moteur_AVD");
        Moteur_ARD = hardwareMap.get(DcMotor.class, "Moteur_ARD");



        Moteur_AVG.setDirection(DcMotor.Direction.REVERSE);

        Moteur_ARG.setDirection(DcMotor.Direction.FORWARD);
        Moteur_AVD.setDirection(DcMotor.Direction.REVERSE);
        Moteur_ARD.setDirection(DcMotor.Direction.REVERSE);

        initialiser_power();

        telemetry.addData("Moteur deplacement initialisation", "OK");




    }

    /**
     * Definir la puissance des moteurs de deplacement
     * si plusieur appel de cette fonction, on fait la moyenne des deux dernieres valeurs pour lisser les commandes
     * @param powerAVG Puissance moteur avant gauche
     * @param powerARG Puissance moteur arriere gauche
     * @param powerAVD Puissance moteur avant droit
     * @param powerARD Puissance moteur arriere droit
     */
    public void setPower(double powerAVG, double powerARG, double powerAVD, double powerARD) {
        this.powerARG = powerARG;
        this.powerAVG = powerAVG;
        this.powerAVD = powerAVD;
        this.powerARD = powerARD;

        //on fait la moyenne des deux derniers appels pour lisser les commandes
        if (!premiereSetPower) {
            this.powerARG = this.powerARG / 2;
            this.powerAVG = this.powerAVG / 2;
            this.powerAVD = this.powerAVD / 2;
            this.powerARD = this.powerARD / 2;
        }
        else
        {

            premiereSetPower = true;
        }

    }


    /**
     * Avancer le robot en fonction des puissances definies precedemment
     * Normalise les puissances si besoin (>1)
     * Ajuste les puissances si elles sont trop faibles pour faire bouger les moteurs
     */
    public void avancerRobot() {

        // Normalisation des puissances si elle depasse 1 (mais sa ne devrait jamais arriver)
        double max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(powerARG),
                Math.abs(powerAVG), Math.abs(powerAVD), Math.abs(powerARD)));
        if (max > 1) {
            powerARG /= max;
            powerAVG /= max;
            powerAVD /= max;
            powerARD /= max;
        }

        telemetry.addData("Power AV G/D", JavaUtil.formatNumber(powerAVG, 4, 2) + ", " + JavaUtil.formatNumber(powerAVD, 4, 2));
        telemetry.addData("Power AR G/D", JavaUtil.formatNumber(powerARG, 4, 2) + ", " + JavaUtil.formatNumber(powerARD, 4, 2));


        // Si la puissance est trop faible, on l'augmente pour eviter que les moteurs ne bougent pas
        double puissance_minimum = 0.08;
        if (powerAVG< puissance_minimum && powerARG< puissance_minimum && powerAVD< puissance_minimum && powerARD< puissance_minimum )
        {
            double multiplicateur = 2;
            powerAVG = powerAVG * multiplicateur;
            powerARG = powerARG * multiplicateur;
            powerAVD = powerAVD * multiplicateur;
            powerARD = powerARD * multiplicateur;
        }

        // Appliquer la puissance aux moteurs
        Moteur_AVG.setPower(powerAVG * multiplicateurPuissance);
        Moteur_ARG.setPower(powerARG * multiplicateurPuissance);
        Moteur_AVD.setPower(powerAVD * multiplicateurPuissance);
        Moteur_ARD.setPower(powerARD * multiplicateurPuissance);

        //
        initialiser_power();
    }

    /**
     * Initialise moteur
     */
    private void initialiser_power() {
        /*
        powerAVG = 0;
        powerARG= 0;
        powerAVD= 0;
        powerARD= 0;
         */


        premiereSetPower = true;
    }
}
