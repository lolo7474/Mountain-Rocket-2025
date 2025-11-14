package org.firstinspires.ftc.teamcode.vittoz.p03_vision_par_camera;

/**
 * Classe pour stocker les informations d'un AprilTag détecté
 */
public class AprilTag {
    public int id;
    public boolean donnees_metriques_disponibles;
    public double range;      // distance (cm)
    public double bearing;    // angle horizontal (°)
    public double yaw;        // rotation (°)
    public double elevation;  // angle vertical (°)
    public double pitch;     // rotation (°)
    public double roll;      // rotation (°)
    public String nom;


    /**
     * Constructor avec toutes les données métriques
     * Les angles sont en degrés et les distances en cm
     * @param id
     * @param range // distance (cm)
     * @param bearing // angle horizontal (°)
     * @param elevation // angle vertical (°)
     * @param yaw // angle du tag par rapport à la caméra (°)
     * @param pitch
     * @param roll
     */
    public AprilTag(int id, double range, double bearing, double elevation, double yaw, double pitch, double roll) {

        /*
        telemetry.addData("Range" , tag.ftcPose.range * rangeCorrection);
                    telemetry.addData("Bearing" , tag.ftcPose.bearing );
                    telemetry.addData("Elevation" , tag.ftcPose.elevation);
                    telemetry.addData("yaw" , tag.ftcPose.yaw);
                    telemetry.addData("Pitch" , tag.ftcPose.pitch);
                    telemetry.addData("Roll" , tag.ftcPose.roll);
         */
        this.id = id;
        this.range = range ;
        this.bearing = bearing ;
        this.yaw = yaw;
        this.elevation = elevation;
        this.pitch = pitch;
        this.roll = roll;
        this.donnees_metriques_disponibles = true;

        attribuerNomTag(id);


    }

    /**
     * Constructor sans données métriques
     * @param id
     */
    public AprilTag(int id) {
        this.id = id;
        this.donnees_metriques_disponibles = false;
        attribuerNomTag(id);
    }



    /**
     * Attribue un nom au tag en fonction de son ID
     * @param id
     */
    private void attribuerNomTag(int id) {
        if (id == 24)
        {
            this.nom = "But Rouge";
        }
        else if (id == 20) {
            this.nom = "But Bleu";
        }
        else if (id == 22)
        {
            this.nom = "Obelisk PGP";
        }else if (id == 23)
        {
            this.nom = "Obelisk PPG";
        }else if (id == 21)
        {
            this.nom = "Obelisk GPP";
        }
        else
        {
            this.nom = "Inconnu";
        }
    }

    public String toString() {
        if (donnees_metriques_disponibles) {
            return "AprilTag "+nom+"{" +""+
                    "   id=" + id + ""+
                    "   dist:" + String.format("%.2f", range) +""+
                    "   bearing=" + String.format("%.2f", bearing) +""+
                    //", elevation=" + String.format("%.2f", elevation) +"\n"+
                    "   yaw:" + String.format("%.2f", yaw) + ""+
                    //", pitch=" + String.format("%.2f", pitch) +"\n"+
                    //", roll=" + String.format("%.2f", roll) +"\n"+
                    '}';
        } else {
            return "AprilTag \"+nom+\"{" +
                    "\nid=" + id +
                    "\n   metriques_disponibles" + "\n"+
                    '}';
        }
    }

}