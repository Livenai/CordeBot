if [ $# -eq 0 ]
  then
    echo "El Script necesita como argumento el numero de segundos a esperar entre despertar una oveja y otra."
    echo "Ej: ./script.sh 5"
fi
if [ $# -gt 1 ]
  then
    echo "El Script solo necesita un parametro. Ejecutelo sin parametros para mas informacion."
fi
if [ $# -eq 1 ]
  then
    /bin/MyFirstComp /home/carlos/robocomp/components/robotsOvejas/oveja/etc/config &
    sleep $1s
    /bin/MyFirstComp /home/carlos/robocomp/components/robotsOvejas/oveja/etc/config1 &
    sleep $1s
    /bin/MyFirstComp /home/carlos/robocomp/components/robotsOvejas/oveja/etc/config2 &
    sleep $1s
    /bin/MyFirstComp /home/carlos/robocomp/components/robotsOvejas/oveja/etc/config3 &
    sleep $1s
    /bin/MyFirstComp /home/carlos/robocomp/components/robotsOvejas/oveja/etc/config4 &
fi
