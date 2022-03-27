#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>

// Qui includo cose utili, problemi nell'includere il service (problemi di dipendenze fra packages).
#include <geometry_msgs/PoseStamped.h>
#include "diffdrive_kin_ctrl/GenerateDesiredPathService.h"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "dwa_demo");

    tf2_ros::Buffer tfBuffer(ros::Duration(10));
    tf2_ros::TransformListener tfListener(tfBuffer);
        
    costmap_2d::Costmap2DROS my_costmap("my_costmap", tfBuffer);

    dwa_local_planner::DWAPlannerROS dp;
    dp.initialize("my_dwa_planner", &tfBuffer, &my_costmap);
    
    // DA QUI COSE NUOVE
    
    /*
     * Come usare DWA con un nostra path come reference (dal service).
     * 
    1) Ottieni path di riferimento chiamando servizio eight_trajgen e trasfomalo in un vettore di PoseStamped.
    	1.a) Passa il path al planner usando setPlan().
    		 setPlan() prende come argomento un vettore di PoseStamped.
    
    2) Crea variabile di tipo PoseStamped e Twist.
       PoseStamped va riempito con position, orientation e velocity correnti.
       Twist sarà riempito con le nuove velocità --> sia lineare che angolare.
       2.a) (forse) dividi la omega in omega_dx e omega_sx.
    
    3) Chiama dwaComputeVelocityCommands() passandogli i parametri appena dichiarati.
    
    4) Usa velocità ottenute per calcolare il nuovo stato [x, y, theta] come diffdrive_kin_ode.
    
    5) Ripeti da punto 2 fino a fine path.
       La fine del path forse la si trova con isGoalReached().
    */
    
    // Definisco un vettore di PoseStamped, forse qui un aiuto:
    // https://answers.ros.org/question/64547/how-to-subscribe-vectorgeometry_msgsposestamped-type-topic/
    /* geometry_msgs::PoseStamped refPath; */
    
    // Da qui definisco gli elementi per usare il service che genera il path di riferimento a otto.
    ros::ServiceClient client;
    diffdrive_kin_ctrl::GenerateDesiredPathService srv;
    std::vector<double> xref_vector, yref_vector;
    ros::NodeHandle Handle;
    
    client = Handle.serviceClient<diffdrive_kin_ctrl::GenerateDesiredPathService>("generate_desired_path_service");
    
    while (!client.call(srv)) {
        ROS_INFO("Waiting for service");
    }
    
    for (uint t = 0; t < srv.response.xref.size(); t++) {
        xref_vector.push_back(srv.response.xref[t]);
        yref_vector.push_back(srv.response.yref[t]);            
    }
    
    ROS_INFO("DEMO.CPP -> Path has been generated and received.");
    
    // Creo il vettore di PoseStamped usando le xref e yref --> pseudocodice, così non va
    /*for (uint i = 0; i < xref_vector.size(); i++){
    	refPath.add(xref_vector[i], yref_vector[i]);
    }
    
    // Imposto il path.
    setPlan(refPath);
    
    // Definisco variabili utili per seguire la traiettoria
    geometry_msgs::PoseStamped currPos;
    geometry_msgs::Twist newLinAngVel;
    
    uint i = 0;
    
    //Inizializza currPos con position, orientation e velocity di partenza.
    currPos = ....
    
    // Uso un while per scorrere tutti i punti del path di riferimento.
    while(i < refPath.size()){
    	
    	dwaComputeVelocityCommands(currPos, newLinAngVel);
    	
    	//Chiama diffdrive_kin_ode o qualcosa di simile per calcolare il nuovo stato (x, y, theta) che si ha con le nuove velocità.
    	
    }*/
     
     
     

    return (0);

}
