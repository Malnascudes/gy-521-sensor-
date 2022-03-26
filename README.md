# Sensores:
el codigo bueno es [`esp8266_gy521_ware`](https://github.com/Malnascudes/gy-521-sensor-/blob/master/esp8266_gy521_ware/esp8266_gy521_ware.ino). Tengo que hacer limpieza del respoitorio

mandan:
- aceleración
- gyroscope data
- pitch/roll/yawn: Valores que se calculan a partir de la acelaración y valores de giroscopio https://www.researchgate.net/publication/329603549/figure/fig1/AS:703246263926788@1544678383831/An-illustration-of-the-three-angles-yaw-pitch-and-roll-returned-by-the-Hyper-IMU.png 

por el puerto 8000 i 8001 (uno para cada sensor) a la IP local del ordenador (pe. 192.168.1.108). 
**HAY QUE CONFIGURARLO UNA VEZ TENGAMOS EL WIFI Y SEPAMOS A QUE ORDENADOR SE VA A MANDAR**

usa [mensajes OSC](https://en.wikipedia.org/wiki/Open_Sound_Control) ([Ejemplo](http://osw.sourceforge.net/html/osc.htm))

Son mensajes que se mandan por internet (en este caso usando UDP, linea 421 del codigo) parametros usando rutas usando “/“ para separarlas. Por ejemplo:
/instrumento_1/delay/feedback 80

o en nuestro caso (linia 399 del codigo)
/acc/x aceleracion_en_el_eje_x
/acc/y aceleracion_en_el_eje_y
/acc/z aceleracion_en_el_eje_z
/gyr/x inclinacion_eje_x
/gyr/y inclinacion_eje_y
/gyr/z inclinacion_eje_z
/yawn valor_yawn
/pitch valor_pitch
/roll  valor_roll


# PATCH de max

Parsea los mensajes OSC de manera que separa el valor de cada parametro para poder usarlo por separado (osea mira los que tienen /acc/x y coge el numero que hay luego y asi para cada uno) y puede hacer con cada uno:
- Hacer TRIGGER de una nota cada vez que un valor pasa por encima de un threshold superior o cuando pasa por debajo de un threshold inferior. Manda una nota distinta para cada threshold)
- Cambiar los valores de un knob dentro del device que:
    - puede controlar cualquier parametro de ableton
    - se puede escoger el valor minimo y maximo de entrada (osea ajustar el range al que responde el knob)
    - si es necesario se podria hacer que mandara mensajes MIDI CC para controlar parametros via MIDI
- Cambia los valores un knob INVERSO dentro del device (osea que esta a maximo cuando el valor es minimo):
    - hace lo mismo que el otro solo que al reves
    - no se puede controlar el range ya que este knob es un slave del otro
    - osea que hay 2 knobs para cada valor

de momento no estan todas las cosas montadas para todos los valores, por ejemplo los triggers solo se hacen con valores de aceleracion, pero se puede montar para que pase facilmente.
el device ahora mismo es bastante loco y hace mas cosas de las que van a hacer falta, yo haria una version reducida para hacer el show que tubiera solo lo necesario
