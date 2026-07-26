#pragma once   // evita inclusioni doppie — alternativa moderna agli include guard

#include <string>


//costanti
constexpr double eps_floating = 0.0001; // piccola elevazione per evitare problemi di collision detection con il piano del tavolo



/* POOL TABLE*/

    //id
    const std::string ID_MINI_POOL_TABLE = "mini_pool_table";


    //dimensioni del campo (senza sponde)*
    constexpr double pool_table_field_length = 0.45;   // lunghezza del campo (in metri)
    constexpr double pool_table_field_width = 0.275;   // larghezza del campo (in metri)
    constexpr double pool_table_field_height = 0.060;   // altezza del campo (in metri)
    //*Nota: utile per la generazione casuale delle palline (todo futuro)


    //terne biliardo 
    const std::string TERNA_RIFERIMENTO_BILIARDO = "world";                                 //posizionamento si riferisce al mondo
    const std::string TERNA_SOLIDALE_CENTRO_BILIARDO = "pool_table_center_frame";           //nome della terna solidale al CENTRO biliardo
    //const std::string TERNA_SOLIDALE_ANGOLO_SX_BILIARDO = "pool_table_angle_frame";       //nome della terna solidale ad un angolo del biliardo


    //posizionmento mini-tavolo da biliardo (rispetto a world)
    constexpr double mini_pool_table_x = 0.0;
    constexpr double mini_pool_table_y = -0.20;
    constexpr double mini_pool_table_z = eps_floating; // appoggio sul piano del tavolo del biliardo



/*BALLS*/

    //id
    const std::string ID_WHITE_BALL = "white_ball";
    const std::string ID_RED_BALL = "red_ball";
    const std::string ID_BLUE_BALL = "blue_ball";
    const std::string ID_YELLOW_BALL = "yellow_ball";


    //terne
    const std::string TERNA_RIFERIMENTO_PALLINE = TERNA_SOLIDALE_CENTRO_BILIARDO;       // posizionamento delle palline si riferisce al centro del biliardo


    //misure
    constexpr double ball_radius = 0.0125;                  // raggio pallina da biliardo (in metri)
    constexpr double Z_BALL = ball_radius + eps_floating;   // altezza = const, appoggiata sul piano


    //posizionamento iniziale pallina bianca (rispetto a centro del biliardo)
    constexpr double w_ball_x = 0.0;                // al centro del biliardo
    constexpr double w_ball_y = 0.0;                // al centro del biliardo
    constexpr double w_ball_z = Z_BALL;             // appoggio sul biliardo

    //posizionamento iniziale pallina rossa (rispetto a centro del biliardo)
    constexpr double r_ball_x = 0.07;               // spostata di 7 cm lungo x rispetto al centro del biliardo
    constexpr double r_ball_y = 0.05;               // spostata di 5 cm lungo y rispetto al centro del biliardo
    constexpr double r_ball_z = Z_BALL;             // appoggio sul biliardo

    //posizionamento iniziale pallina blue (rispetto a centro del biliardo)
    constexpr double b_ball_x = -0.08;               // spostata di -8 cm lungo x rispetto al centro del biliardo
    constexpr double b_ball_y = -0.05;               // spostata di -5 cm lungo y rispetto al centro del biliardo
    constexpr double b_ball_z = Z_BALL;              // appoggio sul biliardo

    //posizionamento iniziale pallina gialla (rispetto a centro del biliardo)
    constexpr double y_ball_x = 0.15;                // spostata di 15 cm lungo x rispetto al centro del biliardo
    constexpr double y_ball_y = -0.03;                // spostata di -3 cm lungo y rispetto al centro del biliardo
    constexpr double y_ball_z = Z_BALL;              // appoggio sul biliardo
