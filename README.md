# RobotPintor

Se trata de un proyecto donde el diseño de las piezas son de código libre de [MakersBox](https://www.thingiverse.com/MakersBox/about). Tiene dos motores paso a paso que permiten el desplazamiento con gran precisión, un giro del eje completo tiene un número de pasos determinado, de forma que contando los pasos que vamos dando podemos obtener el número de vueltas de la rueda, y sabiendo su perímetro, la distancia avanzada, este método de estimación de la posición se llama odometría, en robótica móvil se suelen añadir otro tipo de sensores para mejorar la información del posicionamiento, ya que de por sí solo tiene a desviarse ya que cualquier error, por pequeño que sea, se acumula.
Un rotulador en el centro acoplado a un servomotor, permite dibujar a medida que avanzamos, pudiendo escribir textos y realizar figuras.
Para memorizar el recorrido de cada una de las letras se le han introducido los puntos de coordenadas por los que tiene que pasar, el programa se encarga de traducir esos puntos en distancia y ángulo a girar.