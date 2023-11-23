# Arduino_UNO_WIFI_ESTACION_METEOROLOGICA

Internet de las Cosas (IoT) utilizando Arduino UNO WIFI y MQTT para leer datos de diversos sensores. Este curso está diseñado para proporcionarles una comprensión sólida de cómo integrar la tecnología Arduino WIFI con el protocolo MQTT para crear soluciones IoT robustas y eficientes.

## Paso 1: Descagar o Clonar

Descarga o clona el proyecto para que quede en su ordenador.


## Paso 2: Eleccion de placa

- En el gestor de tarjetas instale: Arduino megaAVR Boards.
- Seleccione la placa: Arduino UNO WIFI Rev2.

## Contribución

Este proyecto ha sido posible gracias a la colaboración de Andrés Hernández Chaves e Ignacio Cordero Chinchilla durante la práctica supervisada del año 2023.

## Problemas o Sugerencia
____________________________________________________________________
- Los pasos en este README, son para utilizar el programa en el IDE de arduino, por lo que es recomendable el uso de este programa.
_____________________________________________________________________
Si usted se encuentra en linux y no permite subir la programacion a la placa de Arduino UNO WIFI aqui le dejamos una posible solucion:  

- ERROR DE PERMISOS AL SUBIR SCKETCH EN EL PUERTO tty/ACM0.

Arduino:1.8.19 (Linux), Tarjeta:"Arduino Uno WiFi Rev2, ATMEGA328"
_______________________________________________________________________
* ERROR:
Ha ocurrido un error mientras se enviaba el sketch
avrdude: usbdev_open(): cannot open device: Permission denied
avrdude: jtag3_open_common(): Did not find any device matching VID 0x03eb and PID list: 0x2145.


* SOLUCIÓN:

sudo chmod a+rw /dev/ttyACM0
pi@raspberrypi:~ $ sudo usermod -a -G uucp pi
pi@raspberrypi:~ $ sudo udevadm control --reload-rules && sudo udevadm trigger
pi@raspberrypi:~ $ 
________________________________________________________________________
