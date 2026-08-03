#pragma once   // evita inclusioni doppie — alternativa moderna agli include guard
#include <string>



/*NOMI TERNE AMBIENTE*/
    const std::string WORLD_FRAME = "world";                            // nome della terna di riferimento globale (world frame)

    const std::string BILLIARD_TABLE_FRAME = "billiard_center_field";   // nome della terna del tavolo da biliardo

    const std::string SOLID_BALL_FRAME = "solid_ball";          //nome terna palline piene, da completare con prefisso*              
    //const std::string STRIPES_BALL_FRAME = "stripes_ball";    //nome terna palline a strisce, da completare con prefisso*      
    //*es red --> red_solid_ball, red_stripes_ball ecc..


    /*NOTA: in questo progetto usiamo solo le seguenti palline piene*/

    const std::string WHITE_SOLID_BALL_FRAME = "white_solid_ball";
    const std::string RED_SOLID_BALL_FRAME = "red_solid_ball";
    const std::string BLUE_SOLID_BALL_FRAME = "blue_solid_ball";
    const std::string YELLOW_SOLID_BALL_FRAME = "yellow_solid_ball";


/* POOL TABLE*/

    //id
    const std::string ID_MINI_POOL_TABLE = "mini_pool_table";


    //dimensioni del campo (senza sponde)
    constexpr double POOL_TABLE_FIELD_LENGTH = 0.45;   // lunghezza del campo (in metri)
    constexpr double POOL_TABLE_FIELD_WIDTH = 0.275;   // larghezza del campo (in metri)
    constexpr double POOL_TABLE_FIELD_HEIGHT = 0.060;   // altezza del campo (in metri)
    

    //posizionmento mini-tavolo da biliardo (rispetto a world)
    // constexpr double mini_pool_table_x = 0.0;
    // constexpr double mini_pool_table_y = -0.20;
    // constexpr double mini_pool_table_z = eps_floating; // appoggio sul piano del tavolo del biliardo



/*BALLS*/

    //id palline di lavoro (UGUALI AI NOMI DEI FRAME PER SEMPLICITA)*
    const std::string ID_WHITE_SOLID_BALL = WHITE_SOLID_BALL_FRAME;
    const std::string ID_RED_SOLID_BALL = RED_SOLID_BALL_FRAME;
    const std::string ID_BLUE_SOLID_BALL = BLUE_SOLID_BALL_FRAME;
    const std::string ID_YELLOW_SOLID_BALL = YELLOW_SOLID_BALL_FRAME;
    //* in futuro si possono aggiungere altre palline, ma per ora ci bastano queste 4
    

    //misure
    constexpr double BALL_RADIUS = 0.0125;                  // raggio pallina da biliardo (in metri)


    // constexpr double Z_BALL = BALL_RADIUS;   // altezza = const, appoggiata sul piano


    // //posizionamento iniziale pallina bianca (rispetto a centro del biliardo)
    // constexpr double w_ball_x = 0.0;                // al centro del biliardo
    // constexpr double w_ball_y = 0.0;                // al centro del biliardo
    // constexpr double w_ball_z = Z_BALL;             // appoggio sul biliardo

    // //posizionamento iniziale pallina rossa (rispetto a centro del biliardo)
    // constexpr double r_ball_x = 0.07;               // spostata di 7 cm lungo x rispetto al centro del biliardo
    // constexpr double r_ball_y = 0.05;               // spostata di 5 cm lungo y rispetto al centro del biliardo
    // constexpr double r_ball_z = Z_BALL;             // appoggio sul biliardo

    // //posizionamento iniziale pallina blue (rispetto a centro del biliardo)
    // constexpr double b_ball_x = -0.08;               // spostata di -8 cm lungo x rispetto al centro del biliardo
    // constexpr double b_ball_y = -0.05;               // spostata di -5 cm lungo y rispetto al centro del biliardo
    // constexpr double b_ball_z = Z_BALL;              // appoggio sul biliardo

    // //posizionamento iniziale pallina gialla (rispetto a centro del biliardo)
    // constexpr double y_ball_x = 0.15;                // spostata di 15 cm lungo x rispetto al centro del biliardo
    // constexpr double y_ball_y = -0.03;                // spostata di -3 cm lungo y rispetto al centro del biliardo
    // constexpr double y_ball_z = Z_BALL;              // appoggio sul biliardo
