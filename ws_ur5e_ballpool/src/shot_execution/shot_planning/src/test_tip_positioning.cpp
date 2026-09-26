// ============================================================
//  test_tip_positioning.cpp

//  Porta il tip dell'asta in punti noti del campo da biliardo
//  (centro-campo + punti attorno ad ogni pallina), per verificare
//  la corrispondenza tra le coordinate stimate dalla camera e
//  la posizione reale raggiunta dal robot.
//
//  Sequenza:
//    1.1) tip → (0, 0, 5 cm) rispetto al centro del campo
//    1.2) tip → (0, 0, 3 cm) rispetto al centro del campo
//    1.3) tip → (0, 0, 2 cm) rispetto al centro del campo
//    1.4) tip → (0, 0, 1 cm) rispetto al centro del campo

//    2) per ogni pallina rilevata:
//         a) tip → (0, 0, R+1  cm)     [sopra il centro pallina]
//         b) tip → (R cm, 0, 0)     [lato +X pallina]
//         c) tip → (0, R cm, 0)     [lato +Y pallina]
//         d) risalita e passaggio alla pallina successiva
//
//  Ogni singolo movimento cartesiano chiede conferma all'utente
//  (tramite print_and_wait) prima di essere eseguito.
//
//  NOTE:
//   - Il moto verso i punti laterali (b) e (c) NON avviene in linea
//     retta dal punto precedente: per evitare che l'asta attraversi
//     la pallina, ogni target viene raggiunto passando prima per una
//     quota di sicurezza (calib_safety_height) sopra il target stesso,
//     e poi scendendo verticalmente. Sono quindi 2 movimenti cartesiani
//     distinti per ogni punto.
//   - L'orientamento usato è: asta verticale puntata verso il basso
// ============================================================

#include "shot_planning/task_node.hpp"

using namespace std::chrono_literals;

namespace
{

// Soglia minima di frazione di traiettoria cartesiana per considerare un
// movimento riuscito. IMPORTANTE: il default di moveCartesianPath() è 0.00,
// il che significa che una traiettoria "trovata" allo 0% (cioè fallita già
// sul primo punto) verrebbe comunque "eseguita" (di fatto un no-op) e
// riportata come successo da MoveIt. Usando qui una soglia alta, i
// fallimenti vengono segnalati chiaramente invece di passare inosservati.
constexpr double CALIB_SUCCESS_THRESHOLD = 0.95;

// Esegue un singolo movimento cartesiano verso "point" (rispetto a frame_id),
// chiedendo prima conferma all'utente. Se il movimento fallisce (fraction
// sotto soglia), stampa un errore ben visibile e chiede all'utente se
// vuole comunque proseguire con il resto della sequenza.
double confirmedMove(const std::shared_ptr<TaskNode>& node,
                      const Vector3d& point,
                      const Quaternion& orientation,
                      const std::string& frame_id,
                      const std::string& description)
{
    node->print_and_wait(
        "\n→ " + description +
        "  [target = (" + std::to_string(point.x()) + ", " +
        std::to_string(point.y()) + ", " + std::to_string(point.z()) +
        ") rispetto a '" + frame_id + "']");

    double fraction = node->moveCartesianPath(point, orientation, frame_id,
                                               CALIB_SUCCESS_THRESHOLD);

    if (fraction < 0.0) {
        RCLCPP_ERROR(node->get_logger(),
                      "MOVIMENTO FALLITO (TF non trovata, target irraggiungibile o non pianificabile): %s",
                      description.c_str());
        node->print_and_wait("  ⚠ Il movimento NON è stato eseguito. Controlla RViz/log prima di continuare.");
    } else if (fraction < CALIB_SUCCESS_THRESHOLD) {
        RCLCPP_ERROR(node->get_logger(),
                      "MOVIMENTO NON ESEGUITO: traiettoria calcolata solo al %.1f%% (sotto soglia %.0f%%): %s",
                      fraction * 100.0, CALIB_SUCCESS_THRESHOLD * 100.0, description.c_str());
        node->print_and_wait("  ⚠ Il movimento NON è stato eseguito. Controlla RViz/log prima di continuare.");
    }

    return fraction;
}

// Movimento "sicuro" verso un punto: prima ci si sposta in orizzontale alla
// quota di sicurezza sopra il target (per non attraversare la pallina),
// poi si scende verticalmente. Due movimenti cartesiani distinti, ciascuno
// con la propria richiesta di conferma.
void safeApproach(const std::shared_ptr<TaskNode>& node,
                   const Vector3d& target,
                   const Quaternion& orientation,
                   const std::string& frame_id,
                   double safety_height,
                   const std::string& label)
{
    Vector3d over_target(target.x(), target.y(), safety_height);

    confirmedMove(node, over_target, orientation, frame_id,
                  label + " - spostamento in aria alla quota di sicurezza");

    confirmedMove(node, target, orientation, frame_id,
                  label + " - discesa verso il punto target");
}

} // namespace


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<TaskNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Aspetta che start() abbia completato l'inizializzazione di moveit
    node->waitInit();

    //------------------------------------------------------
    /* PARAMETRI DI CALIBRAZIONE */
    //node->declare_parameter<double>("calib_offset_from_ball_surface", 0.01);  // 1 cm sopra la superficie pallina
    //node->declare_parameter<double>("calib_offset_from_table_center", 0.01);  // 1 cm sopra il centro campo
    //node->declare_parameter<double>("calib_safety_height", 0.05);             // quota di sicurezza per gli spostamenti in aria

    //double offset_from_ball_surface  = node->get_parameter("calib_offset_from_ball_surface").as_double();
    //double offset_from_table_center  = node->get_parameter("calib_offset_from_table_center").as_double();
    //double safety_height             = node->get_parameter("calib_safety_height").as_double();
    //------------------------------------------------------

    // Costruisco la scena di pianificazione (tavolo, palline, ecc.)
    node->build_scene();

    // Verifico quali terne sono disponibili (solo a scopo informativo/log,
    // non blocca l'esecuzione se manca qualche pallina colorata)
    // node->checkSceneIdentification(WORLD_FRAME);

    //------------------------------------------------------
    /* POSIZIONAMENTO INIZIALE IN CONFIGURAZIONE NOTA
       IMPORTANTE: partire da una configurazione di giunti nota (invece che
       da qualunque posa il robot si trovi ad avere all'avvio) dà a MoveIt
       un seed IK affidabile per i successivi movimenti cartesiani. Senza
       questo passo, computeCartesianPath può fallire già al primo micro-step
       di interpolazione se la posa di partenza è "scomoda" (vicina a una
       singolarità, lontana dal target, ecc.), esattamente come succede nelle
       varie fasi di main.cpp prima di ogni sequenza cartesiana. */
    node->print_and_wait("\n\nPosizionamento in configurazione nota prima di iniziare la calibrazione...");
    node->moveToNamedTarget(AWAY_FROM_TABLE_CONFIG);
    //------------------------------------------------------

    //------------------------------------------------------
    /* ORIENTAMENTO: vogliamo solo che l'asse Z locale del tool (EE_LINK)
       punti verso il basso, cioè lungo -Z del frame di riferimento
       (billiard_center_field / white_solid_ball / ecc.). La rotazione
       attorno a quell'asse (roll dell'asta) è libera e non ci interessa.
       FromTwoVectors calcola la rotazione minima che porta il vettore
       sorgente (Z locale) sul vettore destinazione (-Z), lasciando
       indeterminata ma valida la componente di rotazione attorno all'asse
       di allineamento. */
    Quaternion Q_orient = Quaternion::FromTwoVectors(Vector3d::UnitZ(), Vector3d(0.0, 0.0, -1.0));
    //------------------------------------------------------


    //parametri di esperimento
    double safety_height = 0.05;
    double radius_and_offset = BALL_RADIUS + 0.01; // Raggio pallina + 1 cm


    {
        //centro del tavolo
        //------------------------------------------------------
        // 1.1) Centro field: (0, 0, 5cm) rispetto al tavolo
        //------------------------------------------------------
        node->print_and_wait("\n\nInizio sequenza di prova. Primo punto: centro campo");

        confirmedMove(node, Vector3d(0.0, 0.0, 0.05), Q_orient,
                    BILLIARD_TABLE_FRAME, "Centro campo (5 cm sopra il tavolo)");


        //------------------------------------------------------
        // 1.2) Centro field: (0, 0, 3cm) rispetto al tavolo
        //------------------------------------------------------
        node->print_and_wait("\n\nSecondo punto: centro campo");

        confirmedMove(node, Vector3d(0.0, 0.0, 0.03), Q_orient,
                    BILLIARD_TABLE_FRAME, "Centro campo (3 cm sopra il tavolo)");

        
        //------------------------------------------------------
        // 1.3) Centro field: (0, 0, 2cm) rispetto al tavolo
        //------------------------------------------------------
        node->print_and_wait("\n\nTerzo punto: centro campo");

        confirmedMove(node, Vector3d(0.0, 0.0, 0.02), Q_orient,
                    BILLIARD_TABLE_FRAME, "Centro campo (2 cm sopra il tavolo)");

        //------------------------------------------------------
        // 1.4) Centro field: (0, 0, 1cm) rispetto al tavolo
        //------------------------------------------------------
        node->print_and_wait("\n\nQuarto punto: centro campo");

        confirmedMove(node, Vector3d(0.0, 0.0, 0.01), Q_orient,
                    BILLIARD_TABLE_FRAME, "Centro campo (1 cm sopra il tavolo)");
    }
    

    {
        //------------------------------------------------------
        // 2) Sequenza per ogni pallina
        //------------------------------------------------------
        struct BallEntry { std::string frame; std::string label; };
        const std::vector<BallEntry> balls = {
            { WHITE_SOLID_BALL_FRAME,  "Pallina bianca" },
            { RED_SOLID_BALL_FRAME,    "Pallina rossa"  },
            { BLUE_SOLID_BALL_FRAME,   "Pallina blu"    },
            { YELLOW_SOLID_BALL_FRAME, "Pallina gialla" },
        };

        for (const auto& ball : balls)
        {
            RCLCPP_INFO(node->get_logger(), "\n\n==== %s ====", ball.label.c_str());

            // a) sopra il centro pallina: (0, 0, R + 1cm) — già "alto" di suo
            confirmedMove(node, Vector3d(0.0, 0.0, radius_and_offset), Q_orient,
                        ball.frame, ball.label + " - punto sopra il centro (Z)");

            // b) lato +X pallina: (R + 1cm, 0, 0) — passando per quota sicurezza
            safeApproach(node, Vector3d(radius_and_offset, 0.0, 0.0), Q_orient,
                        ball.frame, safety_height, ball.label + " - punto laterale (X)");

            // c) lato +Y pallina: (0, R + 1cm, 0) — passando per quota sicurezza
            safeApproach(node, Vector3d(0.0, radius_and_offset, 0.0), Q_orient,
                        ball.frame, safety_height, ball.label + " - punto laterale (Y)");

            // d) risalita prima di passare alla pallina successiva
            confirmedMove(node, Vector3d(0.0, 0.0, safety_height), Q_orient,
                        ball.frame, ball.label + " - risalita a quota di sicurezza");
        }

    }
    
    RCLCPP_INFO(node->get_logger(), "\n\nSequenza di prova completata.");

    //------------------------------------------------------
    rclcpp::shutdown();
    spinner.join();

    return 0;
}