#pragma once   // evita inclusioni doppie — alternativa moderna agli include guard
#include <string>



/*NOMI TERNE AMBIENTE*/
    const std::string WORLD_FRAME = "world";                            // nome della terna di riferimento globale (world frame)

    const std::string BILLIARD_TABLE_FRAME = "billiard_center_field";   // nome della terna del tavolo da biliardo

    //tf balls
    const std::string SOLID_BALL_FRAME = "solid_ball";          //nome terna palline piene, da completare con prefisso*              
    //const std::string STRIPES_BALL_FRAME = "stripes_ball";    //nome terna palline a strisce, da completare con prefisso*      
    //*es red --> red_solid_ball, red_stripes_ball ecc..

    /*NOTA: in questo progetto usiamo solo le seguenti palline piene*/
    const std::string WHITE_SOLID_BALL_FRAME = "white_solid_ball";
    const std::string RED_SOLID_BALL_FRAME = "red_solid_ball";
    const std::string BLUE_SOLID_BALL_FRAME = "blue_solid_ball";
    const std::string YELLOW_SOLID_BALL_FRAME = "yellow_solid_ball";


    //tf buche
    const std::string HOLE_TOP_RIGHT_FRAME      = "hole_top_right";
    const std::string HOLE_TOP_LEFT_FRAME       = "hole_top_left";
    const std::string HOLE_MID_RIGHT_FRAME      = "hole_mid_right";
    const std::string HOLE_MID_LEFT_FRAME       = "hole_mid_left";
    const std::string HOLE_BOTTOM_RIGHT_FRAME   = "hole_bottom_right";
    const std::string HOLE_BOTTOM_LEFT_FRAME    = "hole_bottom_left";


/* POOL TABLE*/

    //id
    const std::string ID_MINI_POOL_TABLE = "mini_pool_table";


    //dimensioni del campo (senza sponde)
    constexpr double POOL_TABLE_FIELD_LENGTH = 0.45;   // lunghezza del campo (in metri)
    constexpr double POOL_TABLE_FIELD_WIDTH = 0.275;   // larghezza del campo (in metri)
    constexpr double POOL_TABLE_FIELD_HEIGHT = 0.060;   // altezza del campo (in metri)
    

    // attrito volvente panno con le biglie
    constexpr double CLOTH_SLIDING_FRICTION = 0.005; // u_s
    constexpr double GRAVITY = 9.81;                 // m/s^2


/*BALLS*/

    //id palline di lavoro (UGUALI AI NOMI DEI FRAME PER SEMPLICITA)*
    const std::string ID_WHITE_SOLID_BALL = WHITE_SOLID_BALL_FRAME;
    const std::string ID_RED_SOLID_BALL = RED_SOLID_BALL_FRAME;
    const std::string ID_BLUE_SOLID_BALL = BLUE_SOLID_BALL_FRAME;
    const std::string ID_YELLOW_SOLID_BALL = YELLOW_SOLID_BALL_FRAME;
    //* in futuro si possono aggiungere altre palline, ma per ora ci bastano queste 4
    

    //misure
    constexpr double BALL_RADIUS = 0.0125;                  // raggio pallina da biliardo (in metri)


    //masse
    constexpr double WHITE_BALL_MASS = 0.010;                      // massa pallina da biliardo (in kg)
    constexpr double COLORED_BALL_MASS = 0.015;                    // massa pallina da biliardo (in kg)

    //gravità
    constexpr double GRAVITY = 9.81;                               // accelerazione di gravità

    //attributi fisici del panno del tavolo da biliardo
    constexpr double CLOTH_SLIDING_FRICTION = 0.05;                // coefficiente di attrito radente tra panno e pallina da biliardo