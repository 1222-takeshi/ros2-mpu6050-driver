# ros2_mpu6050_driver

Driver de ROS2 para el sensor IMU MPU6050 (acelerómetro + giroscopio), diseñado para su uso con una Raspberry Pi a través de I2C.

## Entorno Probado

| Componente | Versión |
|-----------|---------|
| Raspberry Pi | 3B+, 4 |
| Ubuntu | 22.04 |
| ROS2 | Humble |

## Prerrequisitos

### WiringPi

En Ubuntu 22.04, WiringPi no está disponible a través de apt y debe compilarse desde el código fuente:

```sh
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi
./build
```

Verifique la instalación:

```sh
gpio -v
```

### Habilitar I2C

En Ubuntu 22.04, habilite I2C editando la configuración de arranque directamente (`raspi-config` no está disponible):

```sh
sudo nano /boot/firmware/config.txt
```

Agregue la siguiente línea:

```
dtparam=i2c_arm=on
```

Luego reinicie:

```sh
sudo reboot
```

Instale i2c-tools y agregue su usuario al grupo `i2c` (para evitar el uso de `sudo` en cada comando):

```sh
sudo apt install i2c-tools
sudo usermod -aG i2c $USER
# Cierre sesión y vuelva a entrar para que el cambio de grupo surta efecto
```

Verifique que el MPU6050 sea detectado (dirección predeterminada: `0x68`):

```sh
i2cdetect -y 1
```

## Instalación

```sh
# Crear un espacio de trabajo de ROS2 (omita si ya tiene uno)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clonar este repositorio
git clone https://github.com/1222-takeshi/ros2-mpu6050-driver.git

# Instalar dependencias
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Compilar
colcon build --symlink-install

# Cargar el espacio de trabajo
source install/setup.bash
```

## Uso

```sh
ros2 launch imu_driver mpu6050_driver.launch.xml
```

La tasa de publicación se controla mediante el parámetro `publish_rate_hz`. El valor predeterminado es `100.0` Hz.

Ejemplo de anulación:

```sh
ros2 launch imu_driver mpu6050_driver.launch.xml publish_rate_hz:=200.0
```

### Tópicos Publicados

| Tópico | Tipo | Descripción |
|-------|------|-------------|
| `output` | `sensor_msgs/Imu` | Datos de la IMU utilizando unidades SI de ROS (`rad/s` velocidad angular, `m/s^2` aceleración lineal) |
| `roll_pitch` | `geometry_msgs/Vector3Stamped` | Ángulos de roll y pitch en grados (`x=roll`, `y=pitch`, `z=0`) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Estado de salud del driver para herramientas de diagnóstico de ROS |

### Unidades de la IMU

El tópico `output` sigue las convenciones de unidades estándar de `sensor_msgs/Imu`:

| Campo | Unidad |
|-------|------|
| `angular_velocity` | `rad/s` |
| `angular_velocity_covariance` | `(rad/s)^2` |
| `linear_acceleration` | `m/s^2` |
| `linear_acceleration_covariance` | `(m/s^2)^2` |

Internamente, el MPU6050 está configurado para un rango de giro de ±250 deg/s y un rango de acelerómetro de ±2g. El driver convierte estas unidades brutas del sensor antes de publicar `sensor_msgs/Imu`.

Si utilizó una versión anterior de este driver que publicaba deg/s y g en `output`, elimine los adaptadores de unidades posteriores o desactive sus conversiones de estilo `gyro_in_degrees` / `accel_in_g` para evitar una doble conversión.

El driver no estima la orientación en el tópico `output`, por lo que `orientation_covariance[0]` siempre se establece en `-1.0`.

### Diagnósticos

El driver publica dos estados de diagnóstico:

| Estado | OK | WARN | ERROR |
|--------|----|------|-------|
| `Hardware Status` | I2C inicializado, temperatura normal y todos los ejes activos | Temperatura del chip superior a 70°C o uno o más ejes en espera | I2C no inicializado, falla la lectura de un registro I2C o temperatura del chip superior a 85°C |
| `Data Status` | Muestras de IMU recientes, dentro del rango y en intervalo | Aún no se ha publicado ninguna muestra, la última muestra está obsoleta, fuera del rango esperado o tiene un jitter de intervalo alto | I2C no inicializado o falló la lectura de la última muestra |

Puede inspeccionar los diagnósticos con:

```sh
ros2 topic echo /diagnostics
```

`Data Status` incluye campos de temporización para observación en tiempo de ejecución:

| Campo | Significado |
|-------|---------|
| `Configured publish_rate_hz` | Tasa de publicación solicitada (vuelve al valor predeterminado si los valores son inválidos) |
| `Effective publish_rate_hz` | Tasa de publicación respaldada por temporizador tras el ajuste de resolución del temporizador de milisegundos |
| `Expected sample interval sec` | Intervalo efectivo del temporizador tras el ajuste de la tasa de publicación |
| `Latest sample interval sec` | Tiempo entre las últimas dos muestras de IMU publicadas |
| `Latest sample interval error sec` | Diferencia absoluta entre los intervalos más reciente y esperado |
| `Max sample interval error sec` | Error de intervalo máximo observado desde el inicio del nodo |
| `Sample interval tolerance sec` | Umbral de advertencia para el error de intervalo |

Para comprobaciones de temporización en el hardware, ejecute el nodo bajo la carga esperada del robot y observe `Latest sample interval error sec` y `Max sample interval error sec` en `/diagnostics`.

### Parámetros

| Parámetro | Tipo | Predeterminado | Descripción |
|-----------|------|---------|-------------|
| `publish_rate_hz` | double | `100.0` | Tasa de publicación de la IMU en Hz |
| `angular_velocity_bias` | double[3] | `[0.0, 0.0, 0.0]` | Sesgo estático del giro a restar de `angular_velocity` en `rad/s` |
| `linear_acceleration_bias` | double[3] | `[0.0, 0.0, 0.0]` | Sesgo estático del acelerómetro a restar de `linear_acceleration` en `m/s^2` |
| `angular_velocity_covariance` | double[9] | `[0.0, ...]` | Covarianza row-major para `angular_velocity`; todo ceros significa desconocido |
| `linear_acceleration_covariance` | double[9] | `[0.0, ...]` | Covarianza row-major para `linear_acceleration`; todo ceros significa desconocido |

Los desfases de sesgo se aplican después del escalado bruto del sensor y la conversión a unidades SI:

```text
published_value = scaled_sensor_value - configured_bias
```

La muestra de aceleración corregida también es utilizada por el tópico `roll_pitch`.

Para una estimación inicial del sesgo estacionario:

1. Coloque la IMU en su orientación de montaje normal y manténgala inmóvil.
2. Grabe una ventana corta de muestras del tópico `output` después del inicio.
3. Use la velocidad angular media de cada eje como `angular_velocity_bias`.
4. Para la aceleración lineal, reste el vector de gravedad esperado para la orientación de montaje de la media medida, y use el residuo como `linear_acceleration_bias`.
5. Mantenga todos los valores en unidades SI de ROS (`rad/s` y `m/s^2`).

Esta es solo una corrección estática. Re-estime los desfases cuando cambie el sensor, el montaje o el entorno operativo.

Los parámetros de covarianza se mapean directamente a las matrices 3x3 row-major en `sensor_msgs/Imu`.
Para una configuración solo diagonal, mantenga los términos fuera de la diagonal en `0.0`:

```sh
ros2 launch imu_driver mpu6050_driver.launch.xml \
  angular_velocity_covariance:="[0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0004]" \
  linear_acceleration_covariance:="[0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]"
```

Para la integración con EKF o filtros, reemplace estos ejemplos con valores derivados de mediciones, suposiciones de la hoja de datos o un procedimiento de calibración. Los valores predeterminados en cero preservan la semántica de covarianza desconocida de ROS y no son valores de estimador ajustados.

### Configuración del Estimador

Use el flujo bruto de `output` como entrada para estimadores posteriores o filtros de orientación.
Con el archivo de lanzamiento proporcionado, `output` se renombra a `/imu/data_raw` por defecto.
Este driver publica la velocidad angular y la aceleración lineal en unidades SI de ROS, pero no estima la orientación. Los consumidores deben tratar `orientation_covariance[0] = -1.0` como orientación no disponible en el flujo bruto.

Secuencia de configuración recomendada:

1. Verifique que el MPU6050 aparezca en I2C con `i2cdetect -y 1`.
2. Inicie el driver y confirme que `/imu/data_raw` publica `sensor_msgs/Imu`.
3. Confirme que `/diagnostics` informe que `Hardware Status` y `Data Status` están en OK bajo carga normal.
4. Estime y configure los desfases de sesgo estático mientras la IMU esté estacionaria.
5. Configure los valores de covarianza a partir de la varianza medida o suposiciones de la hoja de datos.
6. Vuelva a comprobar los campos de temporización de `/diagnostics` mientras el robot ejecuta su carga de trabajo normal.
7. Alimente el filtro o estimador posterior con `/imu/data_raw`.

#### Flujo de Trabajo de Calibración

Ejecute la calibración con la IMU montada en la orientación normal del robot:

1. Mantenga el robot inmóvil sobre una superficie estable.
2. Inicie el driver y espere a que se estabilicen los transitorios de inicio.
3. Grabe una ventana corta de muestras de `/imu/data_raw`.
4. Calcule la media de `angular_velocity` en cada eje y úsela como `angular_velocity_bias`.
5. Calcule la media de `linear_acceleration`, reste el vector de gravedad esperado para la orientación de montaje y use el residuo como `linear_acceleration_bias`.
6. Reinicie el driver con esos valores de sesgo y confirme que la salida estacionaria esté cerca de la velocidad angular cero y cerca del vector de gravedad esperado.

Ejemplo de lanzamiento con sesgo estático y valores de covarianza diagonal:

```sh
ros2 launch imu_driver mpu6050_driver.launch.xml \
  angular_velocity_bias:="[0.001, -0.002, 0.0005]" \
  linear_acceleration_bias:="[0.05, -0.03, 0.10]" \
  angular_velocity_covariance:="[0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0004]" \
  linear_acceleration_covariance:="[0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]"
```

Reemplace los números de ejemplo con valores medidos en su sensor y montaje. Re-estímelos después de cambiar la IMU, el soporte, el cableado, el aislamiento de vibraciones o el entorno operativo.

#### Integración con imu_filter_madgwick

`imu_filter_madgwick` puede estimar la orientación a partir de la velocidad angular y la aceleración, y opcionalmente puede usar datos del magnetómetro. En ROS 2 Humble, instálelo con:

```sh
sudo apt install ros-humble-imu-tools
```

El archivo de lanzamiento de este driver renombra `output` a `/imu/data_raw` por defecto, lo cual coincide con el tópico de entrada de IMU bruta utilizado por `imu_filter_madgwick_node`. Si no tiene un tópico de magnetómetro sincronizado, desactive la fusión del magnetómetro:

```sh
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args \
  -p use_mag:=false \
  -p publish_tf:=false
```

La orientación filtrada se publica en `/imu/data`. Compruébela con:

```sh
ros2 topic echo /imu/data
ros2 topic hz /imu/data
```

Mantenga `remove_gravity_vector` desactivado a menos que el estimador posterior espere explícitamente aceleración lineal compensada por gravedad. Para los estimadores de estado, documente si consumen el flujo bruto `/imu/data_raw` o el flujo filtrado `/imu/data`, ya que solo el flujo filtrado contiene una estimación de la orientación.

#### Lista de Verificación de Temporización y Diagnósticos

Antes de usar la IMU en un EKF, SLAM o bucle de control, verifique:

- `Latest sample age sec` se mantiene por debajo del umbral de muestra obsoleta.
- `Latest sample interval sec` está cerca de `Expected sample interval sec`.
- `Max sample interval error sec` permanece aceptable bajo carga normal de CPU e I2C.
- Los valores de covarianza están configurados cuando el estimador posterior dependa de ellos.
- Los Frame IDs son consistentes con el árbol TF del robot.

### Configuración del Sensor

El driver utiliza los siguientes ajustes predeterminados del MPU6050:

| Parámetro | Valor |
|-----------|-------|
| Rango del Giroscopio | ±250°/s |
| Rango del Acelerómetro | ±2g |
| Tasa de salida | 100 Hz por defecto (`publish_rate_hz`) |
| Dirección I2C | 0x68 |

## Referencias

- [Definición del mensaje sensor_msgs/Imu](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html)
- [Repositorio imu_tools](https://github.com/CCNYRoboticsLab/imu_tools)
- [Paquete imu_filter_madgwick](https://index.ros.org/p/imu_filter_madgwick/)
- [Referencia del driver MPU6050 ROS2](https://shizenkarasuzon.hatenablog.com/entry/2019/03/06/163906)
