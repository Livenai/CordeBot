        ===============================================
        =================   CordeBot   ================
        ===============================================



Componente de Robocomp destinado a la simulacion de corderos/ovejas en un corral virtual, asi como a la simulacion de los sensores de posicion MDEK1001.



Pasos para ejecutar el componente:

  1) Compilar el componente:
    
    (en CordeBot/component)
    cmake .
    make {es posible que senecesite sudo}
    
  2) Ejecutar el mapa:
    
    (en CordeBot/)
    rcis mapa.xml

  3) Ejecutar el shell script RUN_5_LAMBS.sh:
  
    (en CordeBot/oveja)
    ./RUN_5_LAMBS.sh 10  {el 10 es el numero de segundos que el shell espera entre ejecutar una oveja y otra}
  
