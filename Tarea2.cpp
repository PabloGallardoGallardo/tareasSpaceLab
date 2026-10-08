/* Tarea 1: Lectura y transmisión de temperatura y presión

    Conexiones de la BME280 (I2C):
        VDD     => 3V3
        VDDIO   => 3V3
        GND     => GND
        SDA     => D14
        SCL     => D15
        CSB     => 3V3      (CSB a nivel alto => modo I2C)
        SDO     => GND      (SDO a GND => dirección I2C 0x76)

    UART (Serial):
        TX / RX         => pines del puerto Serial de la placa (verificar en el pinout)
        GND             => tierra común
        Baud rate       => 115200
        Bits de datos   => 8
        Paridad         => ninguna
        Bits de parada  => 1        (configuración 8N1)

    Formato de salida:
        TEMP: 23.45 C | PRESS: 101325 Pa
 */

#include <Arduino.h>
#include <Wire.h>

// Dirección I2C del sensor

#define BME280_ADDR       0x76

// Registros de BME280

#define REG_CALIB_START   0x88  // Inicio de los coeficientes de calibración de temperatura y presión (0x88..0x9F)
#define REG_ID            0xD0  // Registro de identificación del chip
#define REG_RESET         0xE0  // Registro de RESET
#define REG_STATUS        0xF3  // Registro de estado del sensor
#define REG_CTRL_MEAS     0xF4  // Registro de control de temperatura y presión (sleep, forzado o normal)
#define REG_CONFIG        0xF5  // Registro de configuración del sensor
#define REG_DATA_START    0xF7  // Registro para el inicio de los datos de medición

// Bits del registro de estado (0xF3)

#define STATUS_MEASURING  0x08  // Bit 3: hay una conversión en curso
#define STATUS_IM_UPDATE  0x01  // Bit 0: se están copiando los datos de calibración desde la NVM

// Constantes

#define BME280_CHIP_ID    0x60      // Valor esperado del chip_id
#define RESET_CMD         0xB6      // Comando de soft reset
#define CTRL_MEAS_SLEEP   0x24      // osrs_t = x1, osrs_p = x1, modo sleep
#define CTRL_MEAS_FORCED  0x25      // osrs_t = x1, osrs_p = x1, modo forzado
#define I2C_CLOCK_HZ      100000    // Frecuencia de reloj I2C (100 kHz, modo estándar)
#define UART_BAUDRATE     115200    // Velocidad de la UART
#define MEASURE_PERIOD_MS 10000     // Periodo de adquisición: 10 s
#define TIMEOUT_MS        100       // Tiempo máximo de espera a que el sensor termine una operación

// Coeficientes de calibración (se leen una sola vez de la NVM del sensor)

static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

static int32_t t_fine;  // Temperatura «fina»: la calcula la compensación de temperatura y la usa la de presión

// Escribe un valor en un registro. Devuelve false si hay error de comunicación

static bool bme280_writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(BME280_ADDR);    // El BME280_ADDR es la direccion, con lo cual nos permite conectarnos directamente al sensor
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;     // 0 => el sensor ha respondido (ACK)
}

// Lee «length» registros consecutivos a partir de «reg» (burst read). Devuelve false si hay error de comunicación

static bool bme280_readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
    Wire.beginTransmission(BME280_ADDR);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom((uint8_t)BME280_ADDR, length) != length) {
        return false;
    }

    // En el bufer cargamos los datos leidos del sensor

    for (uint8_t i = 0; i < length; i++) {

        buffer[i] = Wire.read();

    }

    return true;
}

// Lee un único registro

static bool bme280_readRegister(uint8_t reg, uint8_t *value) {
    return bme280_readRegisters(reg, value, 1);
}

// Espera a que se borren los bits «mask» del registro de estado. Devuelve false si hay error o se agota el tiempo

static bool bme280_waitStatusClear(uint8_t mask) {
    uint32_t start = millis();
    uint8_t status;

    do {
        if (!bme280_readRegister(REG_STATUS, &status)) {
            return false;
        }

        if ((status & mask) == 0) {
            return true;
        }

        delay(1);
    } while (millis() - start < TIMEOUT_MS);

    return false;
}

// Une dos bytes en formato little endian (LSB primero), que es como el sensor guarda la calibración

static uint16_t u16le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// Lee los coeficientes de calibración de temperatura y presión (registros 0x88..0x9F)

static bool bme280_readCalibration() {
    uint8_t c[24];

    if (!bme280_readRegisters(REG_CALIB_START, c, sizeof(c))) {
        return false;
    }

    dig_T1 = u16le(&c[0]);
    dig_T2 = (int16_t)u16le(&c[2]);
    dig_T3 = (int16_t)u16le(&c[4]);
    dig_P1 = u16le(&c[6]);
    dig_P2 = (int16_t)u16le(&c[8]);
    dig_P3 = (int16_t)u16le(&c[10]);
    dig_P4 = (int16_t)u16le(&c[12]);
    dig_P5 = (int16_t)u16le(&c[14]);
    dig_P6 = (int16_t)u16le(&c[16]);
    dig_P7 = (int16_t)u16le(&c[18]);
    dig_P8 = (int16_t)u16le(&c[20]);
    dig_P9 = (int16_t)u16le(&c[22]);

    return true;
}

// Compensación de temperatura (fórmula del datasheet de Bosch). Devuelve la temperatura en centésimas de °C

static int32_t bme280_compensateTemperature(int32_t adc_T) {
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;

    return (t_fine * 5 + 128) >> 8;
}

// Compensación de presión (fórmula de 64 bits del datasheet de Bosch). Devuelve la presión en Pa
// Debe llamarse después de bme280_compensateTemperature(), ya que usa t_fine

static uint32_t bme280_compensatePressure(int32_t adc_P) {
    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

    if (var1 == 0) {
        return 0;   // Evita la división por cero
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);

    return (uint32_t)(p / 256);     // p está en Pa * 256 (formato Q24.8)
}

// =======================================
// Inicialización del sensor
// =======================================

static bool bme280_init() {

    // Verificamos el chip ID y devolvemos un mensaje de verificación

    uint8_t chipId;

    if (!bme280_readRegister(REG_ID, &chipId)) {
        Serial.println("ERROR: el BME280 no responde por I2C.");
        return false;
    }

    if (chipId != BME280_CHIP_ID) {
        Serial.print("ERROR: chip_id incorrecto: 0x");
        Serial.println(chipId, HEX);
        return false;
    }

    Serial.print("Chip ID correcto: 0x");
    Serial.println(chipId, HEX);

    // Soft reset, necesario para garantizar una inicialización «de cero»

    if (!bme280_writeRegister(REG_RESET, RESET_CMD)) {
        return false;
    }

    delay(10);   // >2 ms siguiendo las especificaciones del sensor

    // Esperamos a que termine la copia de la calibración desde la NVM y la leemos

    if (!bme280_waitStatusClear(STATUS_IM_UPDATE) || !bme280_readCalibration()) {
        return false;
    }

    // Filtro IIR desactivado, standby minimo, SPI 3-wire off:
    // es decir, dejamos el sensor en un estado «estable» de reposo, que nos permita partir de una configuración de inicio conocida y adecuada.
    // Se escribe con el sensor en modo sleep, ya que en otros modos la escritura en este registro puede ignorarse

    if (!bme280_writeRegister(REG_CONFIG, 0x00)) {
        return false;
    }

    // Oversampling x1 de temperatura y presión; el sensor queda en sleep hasta que loop() lance la primera medida

    return bme280_writeRegister(REG_CTRL_MEAS, CTRL_MEAS_SLEEP);
}

// =======================================
// Disparo de la medida + burst read
// =======================================

static bool bme280_measure(int32_t *rawTemperature, int32_t *rawPressure) {

    // En forced mode, hay que reescribir «mode» para lanzar una nueva medida

    if (!bme280_writeRegister(REG_CTRL_MEAS, CTRL_MEAS_FORCED)) {
        return false;
    }

    // Con oversampling x1 la medida tarda como máximo ~6,4 ms; esperamos y comprobamos que ha terminado

    delay(10);

    if (!bme280_waitStatusClear(STATUS_MEASURING)) {
        return false;
    }

    // Burst read de presión y temperatura (0xF7..0xFC)

    uint8_t data[6];

    if (!bme280_readRegisters(REG_DATA_START, data, sizeof(data))) {
        return false;
    }

    // Lectura de la presión y de la temperatura (20 bits cada una)

    *rawPressure    = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);
    *rawTemperature = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | (data[5] >> 4);

    // 0x80000 es el valor que devuelve el sensor cuando no se ha realizado la medida

    return (*rawTemperature != 0x80000) && (*rawPressure != 0x80000);
}

// =======================================
// Bucle de inicialización
// =======================================

void setup() {

    Serial.begin(UART_BAUDRATE, SERIAL_8N1);

    // Esperamos al puerto serie (solo afecta a placas con USB nativo; como máximo 3 s para no bloquear sin PC)

    while (!Serial && millis() < 3000) {
        delay(10);
    }

    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);   // 100 kHz (modo estandar)

    Serial.println("Iniciamos lectura del BME280");

    // Si el sensor no responde, reintentamos cada segundo

    while (!bme280_init()) {
        Serial.println("ERROR: no se ha podido inicializar el BME280. Reintentando...");
        delay(1000);
    }

    // Mensajes de confirmación del estado del sensor

    Serial.println("Sensor configurado correctamente.");
    Serial.println("   Oversampling: x1 (temperatura y presión)");
    Serial.println("   Filtro IIR: desactivado");
    Serial.println("   Standby: mínimo");
    Serial.println("   SPI 3-wire: desactivado");

}

// ====================================================================
// Bucle principal: medida + procesado + transmisión por UART cada 10 s
// ====================================================================

void loop() {

    uint32_t start = millis();

    int32_t rawTemperature, rawPressure;

    if (bme280_measure(&rawTemperature, &rawPressure)) {

        // Procesado: conversión de los valores en crudo a °C y Pa con los coeficientes de calibración
        // (la temperatura va primero porque calcula t_fine, que necesita la compensación de presión)

        int32_t  temperature = bme280_compensateTemperature(rawTemperature);   // Centésimas de °C
        uint32_t pressure    = bme280_compensatePressure(rawPressure);         // Pa

        // Salida por UART según el formato solicitado

        Serial.print("TEMP: ");
        Serial.print(temperature / 100.0, 2);
        Serial.print(" C | PRESS: ");
        Serial.print(pressure);
        Serial.println(" Pa");

    }

    else {

        // Si falla la comunicación, lo notificamos y se reintenta en el siguiente ciclo

        Serial.println("ERROR: fallo de comunicación con el BME280.");

    }

    // Esperamos hasta completar los 10 s, descontando lo que ha tardado la medida

    uint32_t elapsed = millis() - start;

    if (elapsed < MEASURE_PERIOD_MS) {

        delay(MEASURE_PERIOD_MS - elapsed);
        
    }

}
