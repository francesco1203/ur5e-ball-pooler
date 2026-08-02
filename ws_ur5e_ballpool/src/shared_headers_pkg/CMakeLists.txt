cmake_minimum_required(VERSION 3.8)
project(shared_headers_pkg)


find_package(ament_cmake REQUIRED)

#pacchetti richiesti per il corretto funzionamento del pacchetto
find_package(Eigen3 REQUIRED) 


# Installa la cartella include affinché sia visibile nel workspace
install(
  DIRECTORY include/
  DESTINATION include
)

#esporto il contenuto del pacchetto, così da renderlo visibile agli altri pacchetti
ament_export_include_directories(include)


ament_package()
