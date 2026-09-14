// ESECUZIONE DEL TIRO

#include "shot_planning/task_node.hpp"

// ── main ────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    /* inizializzazione */
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<TaskNode>();
 
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });
 


    // Aspetta che start() abbia completato l'inizializzazione di moveit — sincronizzazione
    node->waitInit();
    //adesso sono sicuro che start() ha inizializzato move_group_ e posso chiamare i metodi di movimento



    //------------------------------------------------------
    /* SHOT PLANNING PARAMETERS */
    node->declare_parameter<double>("approach_distance_from_ball_surface", 0.02);
    node->declare_parameter<double>("shooting_distance_from_ball_surface", 0.05);
    node->declare_parameter<double>("distance_deceleration_phase_fraction_radius", 2.0);
    // node->declare_parameter<double>("impact_angle_deg", 10.0);           //da motore di gioco
    // node->declare_parameter<double>("direction_angle_deg", 0.0);         //da motore di gioco
    // node->declare_parameter<double>("impact_shot_velocity", 0.1);        //da motore di gioco
    node->declare_parameter<double>("offset_correction_center_z", 0.000);
    node->declare_parameter<double>("elevation_escape", 0.05);
    node->declare_parameter<double>("success_threshold_approach", 0.99);
    node->declare_parameter<double>("success_threshold_back", 0.20);


    double approach_distance_from_ball_surface_ = node->get_parameter("approach_distance_from_ball_surface").as_double();
    double shooting_distance_from_ball_surface_ = node->get_parameter("shooting_distance_from_ball_surface").as_double();
    double distance_deceleration_phase_fraction_radius_ = node->get_parameter("distance_deceleration_phase_fraction_radius").as_double();
    // double impact_angle_deg_ = node->get_parameter("impact_angle_deg").as_double();                  //da motore di gioco
    // double direction_angle_deg_ = node->get_parameter("direction_angle_deg").as_double();            //da motore di gioco
    // double impact_shot_velocity_ = node->get_parameter("impact_shot_velocity").as_double();          //da motore di gioco
    double offset_correction_center_z_ = node->get_parameter("offset_correction_center_z").as_double();
    double elevation_escape_ = node->get_parameter("elevation_escape").as_double();
    double success_threshold_approach_ = node->get_parameter("success_threshold_approach").as_double();
    double success_threshold_back_ = node->get_parameter("success_threshold_back").as_double();
    //------------------------------------------------------


    //------------------------------------------------------
    /*CONTROL EXECUTION PARAMETERS*/
    node->declare_parameter<bool>("control_shot_start_execution_by_user_input", true);
    node->declare_parameter<bool>("control_shot_steps_execution_by_user_input", false);
    node->declare_parameter<bool>("user_confirm_for_right_identification", true);
    
    bool control_shot_start_execution_by_user_input_ = node->get_parameter("control_shot_start_execution_by_user_input").as_bool();
    bool control_shot_steps_execution_by_user_input_ = node->get_parameter("control_shot_steps_execution_by_user_input").as_bool();
    bool user_confirm_for_right_identification_ = node->get_parameter("user_confirm_for_right_identification").as_bool();
    //------------------------------------------------------


    //------------------------------------------------------
    /*USING MUJOCO*/
    node->declare_parameter<bool>("using_mujoco_simulation", false);
    node->declare_parameter<int>("mujoco_sync_pause_time_milliseconds", 800);

    bool using_mujoco_simulation_ = node->get_parameter("using_mujoco_simulation").as_bool();
    int mujoco_sync_pause_time_milliseconds_ = node->get_parameter("mujoco_sync_pause_time_milliseconds").as_int();
    //------------------------------------------------------
    /*MONITORING PARAMETERS debug + logging on file*/

    //log_ruckig = interno al nodo, non è un parametro di configurazione esterno nel main

    node->declare_parameter<bool>("print_EEF_distance_and_position", true);
    bool print_EEF_distance_and_position_ = node->get_parameter("print_EEF_distance_and_position").as_bool();

    node->declare_parameter<bool>("joints_logging_enabled", false);
    node->declare_parameter<bool>("cartesian_logging_enabled", false);
    node->declare_parameter<bool>("torque_logging_enabled", false);
    node->declare_parameter<bool>("controller_logging_enabled", false);

    bool joints_logging_enabled = node->get_parameter("joints_logging_enabled").as_bool();
    bool cartesian_logging_enabled = node->get_parameter("cartesian_logging_enabled").as_bool();
    bool torque_logging_enabled = node->get_parameter("torque_logging_enabled").as_bool() && using_mujoco_simulation_; //solo se sto usando mujoco, altrimenti non funziona
    bool controller_logging_enabled = node->get_parameter("controller_logging_enabled").as_bool();

    node->declare_parameter<bool>("phase_0_logging_enabled", false);    // posizionamento away_from_table
    node->declare_parameter<bool>("phase_1_logging_enabled", false);    // andare in posa pre-approach
    node->declare_parameter<bool>("phase_2_logging_enabled", false);    // approach alla pallina
    node->declare_parameter<bool>("phase_3_logging_enabled", false);    // allontanamento all'indietro per prendere velocità
    node->declare_parameter<bool>("phase_4_logging_enabled", false);    // esecuzione tiro
    node->declare_parameter<bool>("phase_5_logging_enabled", false);    // alzata per liberare il campo

    bool phase_0_logging_enabled = node->get_parameter("phase_0_logging_enabled").as_bool();
    bool phase_1_logging_enabled = node->get_parameter("phase_1_logging_enabled").as_bool();
    bool phase_2_logging_enabled = node->get_parameter("phase_2_logging_enabled").as_bool();
    bool phase_3_logging_enabled = node->get_parameter("phase_3_logging_enabled").as_bool();
    bool phase_4_logging_enabled = node->get_parameter("phase_4_logging_enabled").as_bool();
    bool phase_5_logging_enabled = node->get_parameter("phase_5_logging_enabled").as_bool();
    //------------------------------------------------------


    //------------------------------------------------------
    /* VARIABILI DELL'ESECUZIONE */
    double perc_success;                // percentuale di successo della pianificazione (0.0 - 1.0)
    bool shot_success;                  // flag per indicare se il tiro è stato eseguito con successo
    //------------------------------------------------------



    
    //------------------------------------------------------
    /* SEQUENZA DI TASK */

    
    //inizio
    if(control_shot_start_execution_by_user_input_){
        node->print_and_wait("\n\nPremi un tasto e INVIO per iniziare la sequenza di tiro.. (primo step: away_from_table)");
    }
    else{
        RCLCPP_INFO(node->get_logger(), "\n\nInizio sequenza di tiro..");
    }


    // FASE 0 - mi scosto dal campo, per far fare l'identificazione della scena alla camera senza ostacoli
    {
        if(control_shot_steps_execution_by_user_input_){
            node->print_and_wait("\n\nPosizionamento in 'away_from_table'..");
        }
        else{
            RCLCPP_INFO(node->get_logger(), "\n\nPosizionamento in 'away_from_table'..");
        }

        //logging
        if(phase_0_logging_enabled) node->startLogging("away_from_table", joints_logging_enabled, cartesian_logging_enabled, torque_logging_enabled, controller_logging_enabled);
        

        node->moveToNamedTarget(AWAY_FROM_TABLE_CONFIG);

        if(using_mujoco_simulation_){
            //questo ritardo indispensabile serve a far sincronizzare mujoco (più lento) con moveit
            node->get_clock()->sleep_for(rclcpp::Duration(std::chrono::milliseconds(mujoco_sync_pause_time_milliseconds_)));

            //ATTENZIONE: se non sto usando MuJoCo, questo sleep per qualche motivo non fa più pianificare e blocca il programma
        }
        
        if(phase_0_logging_enabled) node->stopLogging();
       
    }



    //-------------------------------------------
    /* IDENTIFICAZIONE SCENA DA TELECAMERA */

    //qui la camera deve fare l'identificazione della scena, le do il tempo di farlo prima di procedere con la pianificazione del tiro
    RCLCPP_INFO(node->get_logger(), "\n\nIdentificazione scena in corso..");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if(node->checkSceneIdentification(WORLD_FRAME)){
        if(user_confirm_for_right_identification_){
            node->print_and_wait("\n\nPremi un tasto e INVIO per confermare l'identificazione della scena..");
        }
        
        RCLCPP_INFO(node->get_logger(), "\n\nScena identificata. Procedo con la pianificazione del tiro..");
    }
    else{
        RCLCPP_ERROR(node->get_logger(), "\n\nERRORE: Identificazione scena fallita! Impossibile procedere.");
        return 1;
    }
    //-------------------------------------------



    ///-------------------------------------------
    /* COSTRUZIONE SCENA DI PIANIFICAZIONE SU MOVEIT/RVIZ*/

    node->build_scene();  // costruisco la scena di pianificazione (tavolo, pallina, ecc..)
    //-------------------------------------------



    //------------------------------------------------------
    /* LETTURA MOSSA DI GIOCO*/

    node->start_game_engine(); //avvio game engine

    RCLCPP_INFO(node->get_logger(), "In attesa che arrivino i parametri di tiro...");
    node->waitForParams();  // Aspetta che arrivi qualcosa sui topic di parametri di tiro (da motore di gioco)

    node->stop_game_engine();


    // Adesso posso usarli
    double direction_angle_deg_ = node->getDirectionAngle(); 
    double impact_shot_velocity_ = node->getImpactShotVelocity();
    double impact_angle_deg_ = node->getImpactAngle();
    //------------------------------------------------------



    //------------------------------------------------------
    /* CALCOLO DELL'ORIENTAMENTO STECCA*/
    // orientamento è costante in molte fasi, dall'approach all'esecuzione tiro.. lo calcolo una sola volta

    // matrice di rotazione di base che allinea z' -> x, y' --> -y, x' --> -z (da posa di pre approach ad approach base)
    Matrix3d R_base;
    R_base <<  0,  0, -1,
               0, -1,  0,
              -1,  0,  0;

    Quaternion Q_base(R_base);  //converto in quaternione


    //direzione d'impatto
    double impact_angle_rad = impact_angle_deg_ * M_PI / 180;           // inclinazione asta -> rotazione attorno asse y (latitudine)
    double direction_angle_rad = direction_angle_deg_ * M_PI / 180;     // direzione asta -> rotazione attorno asse z (longitudine)


    // calcolo il quaternione dell'orientamento stecca
    Quaternion Q_shot = Quaternion(
        RotationAxis(direction_angle_rad, Z_AXIS) *
        RotationAxis(-impact_angle_rad, Y_AXIS)           //- perché per alzarsi dal tavolo, l'asta deve ruotare in senso orario
    ) * Q_base;


    //Risultato: d'ora in avanti Q_shot è l'orientamento per tutte le sequenze di tiro, dall'approach all'esecuzione del tiro stesso
    //------------------------------------------------------



    // FASE 1 - vado in pre-approach per approcciare la pallina
    {
        if(control_shot_steps_execution_by_user_input_){
            node->print_and_wait("\n\nPosizionamento in 'pre_approach..");
        }
        else{
            RCLCPP_INFO(node->get_logger(), "\n\nPosizionamento in 'pre_approach..");
        }

        //logging
        if(phase_1_logging_enabled) node->startLogging("preapproach", joints_logging_enabled, cartesian_logging_enabled, torque_logging_enabled, controller_logging_enabled);
        

        node->moveToNamedTarget(READY_TO_APPROACH_CONFIG);

        if(using_mujoco_simulation_){
            //questo ritardo indispensabile serve a far sincronizzare mujoco (più lento) con moveit
            node->get_clock()->sleep_for(rclcpp::Duration(std::chrono::milliseconds(mujoco_sync_pause_time_milliseconds_)));

            //ATTENZIONE: se non sto usando MuJoCo, questo sleep per qualche motivo non fa più pianificare e blocca il programma
        }
        
        if(phase_1_logging_enabled) node->stopLogging();
       
    }
    
    

    // FASE 2 - approach alla pallina
    {
        if(control_shot_steps_execution_by_user_input_){
            node->print_and_wait("\n\nApproach alla pallina..");
        }
        else{
            RCLCPP_INFO(node->get_logger(), "\n\nApproach alla pallina..");
        }
        

        //distanza desiderata dal centro della pallina (per allontanarsi)
        double desired_distance_from_ball_center = approach_distance_from_ball_surface_ + BALL_RADIUS; // distanza posizionamento dal centro della pallina


        //uso coordinate sferiche per calcolare la posizione in 3D di dove deve andare la punta dell'asta
        Vector3d pos_pre_shot = Vector3d(desired_distance_from_ball_center * cos(impact_angle_rad) * cos(direction_angle_rad) ,
                                         desired_distance_from_ball_center * cos(impact_angle_rad) * sin(direction_angle_rad), 
                                         desired_distance_from_ball_center * sin(impact_angle_rad) + offset_correction_center_z_
                                        );


        //logging
        if(phase_2_logging_enabled) node->startLogging("approach", joints_logging_enabled, cartesian_logging_enabled, torque_logging_enabled, controller_logging_enabled);
        
        perc_success = node->moveCartesianPath(pos_pre_shot, Q_shot, WHITE_SOLID_BALL_FRAME, 
                                                      success_threshold_approach_); //soglia di successo 95%, perché voglio che ci arrivi

        if(using_mujoco_simulation_){
            //questo ritardo indispensabile serve a far sincronizzare mujoco (più lento) con moveit
            node->get_clock()->sleep_for(rclcpp::Duration(std::chrono::milliseconds(mujoco_sync_pause_time_milliseconds_)));

            //ATTENZIONE: se non sto usando MuJoCo, questo sleep per qualche motivo non fa più pianificare e blocca il programma
        }

        if(phase_2_logging_enabled) node->stopLogging();



        if(print_EEF_distance_and_position_) {
            //prima di procedere, stampo la distanza e la posizione relativa tra tip dell'asta e pallina bianca, utile per debug
            node->printEEFDebugInfo();
        }

        //controllo se l'approach è andato a buon fine
        if(perc_success < success_threshold_approach_) {
            RCLCPP_ERROR(node->get_logger(), "\n\nApproach alla pallina fallito: non è stato possibile raggiungere la posizione desiderata con sufficiente precisione.");
            rclcpp::shutdown();
            spinner.join();
            return -1;
        }
    }



    // FASE 3 - si allontana all'indietro per prendere velocità
    {
  
        if(control_shot_steps_execution_by_user_input_){
            node->print_and_wait("\n\nAllontanamento all'indietro per prendere velocità..");
        }
        else{
            RCLCPP_INFO(node->get_logger(), "\n\nAllontanamento all'indietro per prendere velocità..");
        }

        //distanza desiderata dal centro della pallina (per allontanarsi)
        double desired_distance_from_ball_center = shooting_distance_from_ball_surface_ + BALL_RADIUS; // distanza posizionamento dal centro della pallina


        //uso coordinate sferiche per calcolare la posizione in 3D di dove deve andare la punta dell'asta
        Vector3d pos_back_shot = Vector3d(desired_distance_from_ball_center * cos(impact_angle_rad) * cos(direction_angle_rad) ,
                                          desired_distance_from_ball_center * cos(impact_angle_rad) * sin(direction_angle_rad), 
                                          desired_distance_from_ball_center * sin(impact_angle_rad) + offset_correction_center_z_
                                          );

        //logging
        if(phase_3_logging_enabled) node->startLogging("back_shot", joints_logging_enabled, cartesian_logging_enabled, torque_logging_enabled, controller_logging_enabled);
    

        perc_success = node->moveCartesianPath(pos_back_shot, Q_shot, WHITE_SOLID_BALL_FRAME, 
                                                      success_threshold_back_);            

        
        if(using_mujoco_simulation_){
            //questo ritardo indispensabile serve a far sincronizzare mujoco (più lento) con moveit
            node->get_clock()->sleep_for(rclcpp::Duration(std::chrono::milliseconds(mujoco_sync_pause_time_milliseconds_)));

            //ATTENZIONE: se non sto usando MuJoCo, questo sleep per qualche motivo non fa più pianificare e blocca il programma
        }


        if(phase_3_logging_enabled) node->stopLogging();
        


        if(print_EEF_distance_and_position_) {
            //prima di procedere, stampo la distanza tra tip dell'asta e pallina bianca, utile per debug
            node->printEEFDebugInfo();
        }


        //controllo se l'allontanamento è andato a buon fine
        if(perc_success < success_threshold_back_) {
            RCLCPP_ERROR(node->get_logger(), "\n\nAllontanamento all'indietro fallito: non è stato possibile raggiungere la posizione desiderata con sufficiente precisione.");
            rclcpp::shutdown();
            spinner.join();
            return -1;
        }

        
    }


    // FASE 4 - eseguo tiro
    {
       
        if(control_shot_steps_execution_by_user_input_){
            node->print_and_wait("\n\nEsecuzione tiro..");
        }
        else{
            RCLCPP_INFO(node->get_logger(), "\n\nEsecuzione tiro..");
        }


        //parametri del tiro
        double accel_distance = node->getEEFDistance() - BALL_RADIUS;                       // distanza di accelerazione (dalla posizione all'indietro a cui sono riuscito ad arrivare, fino al contatto con la pallina)
        double decel_distance = distance_deceleration_phase_fraction_radius_ * BALL_RADIUS; // distanza di decelerazione (dal contatto al centro della pallina, scelta progettuale)

        //considerato che decel_distance è la distanza tra il tip dell'asta e il centro della pallina, per calcolare la posizione di arresto devo sottrarre il raggio della pallina
        //distanza arresto = distanza di decelerazione - raggio della pallina
        
        Vector3d pos_arresto = Vector3d(      
                                          -(decel_distance - BALL_RADIUS) * cos(impact_angle_rad) * cos(direction_angle_rad) ,
                                          -(decel_distance - BALL_RADIUS) * cos(impact_angle_rad) * sin(direction_angle_rad), 
                                          -(decel_distance - BALL_RADIUS) * sin(impact_angle_rad) + offset_correction_center_z_
                                        );

        //disabilito collisione tra asta e pallina bianca, così la stecca può penetrare la pallina senza che MoveIt! blocchi il tiro per collisione
        node->disable_white_ball_collision();

        
        //logging
        if(phase_4_logging_enabled) node->startLogging("shot", joints_logging_enabled, cartesian_logging_enabled, torque_logging_enabled, controller_logging_enabled);
     
        shot_success = node->ExecuteShot(pos_arresto, Q_shot, WHITE_SOLID_BALL_FRAME, 
                          impact_shot_velocity_,
                          accel_distance, decel_distance);

        
        if(using_mujoco_simulation_){
            //questo ritardo indispensabile serve a far sincronizzare mujoco (più lento) con moveit
            node->get_clock()->sleep_for(rclcpp::Duration(std::chrono::milliseconds(mujoco_sync_pause_time_milliseconds_)));

            //ATTENZIONE: se non sto usando MuJoCo, questo sleep per qualche motivo non fa più pianificare e blocca il programma
        }


        if(phase_4_logging_enabled) node->stopLogging();
    

        if(print_EEF_distance_and_position_) {
            //prima di procedere, stampo la distanza tra tip dell'asta e pallina bianca, utile per debug
            node->printEEFDebugInfo();
        }
    }


    // FASE 5 - mi alzo un po' per liberare il campo (ma lo faccio solo se ho fatto il tiro)
    {
    
        if(shot_success) {

            if(control_shot_steps_execution_by_user_input_){
                node->print_and_wait("\n\nMi alzo..");
            }
            else{
                RCLCPP_INFO(node->get_logger(), "\n\nMi alzo..");
            }


            //uso coordinate sferiche per calcolare la posizione in 3D di dove deve andare la punta dell'asta
            Vector3d pos_back_shot = Vector3d(0, 0, 0 + elevation_escape_);

            //logging
            if(phase_5_logging_enabled) node->startLogging("get_high", joints_logging_enabled, cartesian_logging_enabled, torque_logging_enabled, controller_logging_enabled);

            node->moveCartesianPath(pos_back_shot, Q_shot, WHITE_SOLID_BALL_FRAME);

            if(using_mujoco_simulation_){
                //questo ritardo indispensabile serve a far sincronizzare mujoco (più lento) con moveit
                node->get_clock()->sleep_for(rclcpp::Duration(std::chrono::milliseconds(mujoco_sync_pause_time_milliseconds_)));

                //ATTENZIONE: se non sto usando MuJoCo, questo sleep per qualche motivo non fa più pianificare e blocca il programma
            }

            if(phase_5_logging_enabled) node->stopLogging();


            if(print_EEF_distance_and_position_) {
                //prima di procedere, stampo la distanza tra tip dell'asta e pallina bianca, utile per debug
                node->printEEFDebugInfo();
            }
        }
        else {
            RCLCPP_WARN(node->get_logger(), "\n\nTiro non eseguito, salto la fase di alzata.");
        }

    }
    

    //----------------------------------------
    // Termina: shutdown sblocca lo spinner, poi join aspetta che finisca
    rclcpp::shutdown();
    spinner.join();
 

    return 0;

}