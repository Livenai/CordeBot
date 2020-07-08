if [ $# -eq 0 ]
  then
    echo "El Script necesita como argumento el numero del cordero a ejecutar (de 0 a 4)."
    echo "Ej: ./script.sh 4"
    echo ""
    echo "[!] Es posible que este script necesite un cambio en la forma de ejecutar el componente en funcion de la maquina en la que se encuentre."
    echo "Revisar y modificar el archivo en caso de error al ejecutar."
fi
if [ $# -gt 1 ]
  then
    echo "El Script solo necesita un parametro. Ejecutelo sin parametros para mas informacion."
fi
if [ $# -eq 1 ]
  then
    /home/robocomp/robocomp/components/CordeBot/navigationComponent/bin/navigationComponent --Ice.Config=/home/robocomp/robocomp/components/CordeBot/navigationComponent/etc/config$1
fi

