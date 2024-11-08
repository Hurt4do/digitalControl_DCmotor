// Pines a los que está conectado el encoder
const int encoderPinA = 36;  // Pin A del encoder (GPIO 36)
const int encoderPinB = 25;  // Pin B del encoder (GPIO 25)

// Pines de control del driver H
const int enablePin = 0;     // Pin de enable del driver H (GPIO 0)
const int in1Pin = 2;       // Pin IN1 del driver H (GPIO 14) - INA
const int in2Pin = 14;        // Pin IN2 del driver H (GPIO 2)  - INB

// Pin del potenciómetro (no utilizado en control de posición PI)
const int potPin = 39;       // Pin del potenciómetro (GPIO 39)

// Configuración PWM
const int pwmChannel = 0;           // Canal PWM (0-15)
const int pwmFreq = 1000;           // Frecuencia PWM en Hz
const int pwmResolution = 8;        // Resolución PWM (8 bits: 0-255)

// Variables de control
float setpoint = 90.0;                // Setpoint para la posición en grados
float error = 0.0;
float prev_error = 0.0;
float u = 0.0;
float prev_u = 0.0;

// Coeficientes de la ecuación de diferencias del controlador PI
const float c1 = 0.11121385;
const float c2 = 0.11118615;

// Variables de posición
float degrees_pos = 0.0;
float radians_pos = 0.0;

// Variables para el conteo de pulsos y dirección
volatile long encoderPulses = 0;       // Contador de pulsos
volatile int lastEncoderAState = LOW;  // Último estado de la señal A
volatile int lastEncoderBState = LOW;  // Último estado de la señal B

// Variables de tiempo para el bucle de control
unsigned long previousMillis = 0;
const unsigned long interval = 10; // Intervalo del bucle de control en milisegundos

/**
 * @brief ISR para manejar los cambios en el pin A del encoder.
 */
void IRAM_ATTR encoderAISR() {
  int currentStateA = digitalRead(encoderPinA);
  int currentStateB = digitalRead(encoderPinB);

  if (currentStateA != lastEncoderAState) {
    if (currentStateA == HIGH) {
      if (currentStateB == LOW) {
        encoderPulses++;
      } else {
        encoderPulses--;
      }
    } else {
      if (currentStateB == HIGH) {
        encoderPulses++;
      } else {
        encoderPulses--;
      }
    }
    lastEncoderAState = currentStateA;
  }
}

/**
 * @brief ISR para manejar los cambios en el pin B del encoder.
 */
void IRAM_ATTR encoderBISR() {
  int currentStateA = digitalRead(encoderPinA);
  int currentStateB = digitalRead(encoderPinB);

  if (currentStateB != lastEncoderBState) {
    if (currentStateB == HIGH) {
      if (currentStateA == HIGH) {
        encoderPulses++;
      } else {
        encoderPulses--;
      }
    } else {
      if (currentStateA == LOW) {
        encoderPulses++;
      } else {
        encoderPulses--;
      }
    }
    lastEncoderBState = currentStateB;
  }
}

/**
 * @brief Configuración inicial del sistema.
 */
void setup() {
  // Configura los pines del encoder como entradas
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  // Configura los pines del driver H como salidas
  pinMode(enablePin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);

  // Configuración inicial del motor: detenido (ambos pines en HIGH para frenado rápido)
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, HIGH);
  ledcWrite(pwmChannel, 0); // PWM apagado inicialmente

  // Inicializa la comunicación serial
  Serial.begin(115200);

  // Adjunta interrupciones a los canales A y B del encoder
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), encoderBISR, CHANGE);

  // Configura el canal PWM
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(enablePin, pwmChannel);
}

/**
 * @brief Bucle principal que ejecuta el control de posición.
 */
void loop() {
  delay(500);
  encoderPulses = 0;
  unsigned long currentMillis = millis();

  // Verifica si ha pasado el intervalo de tiempo para ejecutar el control
  if (currentMillis - previousMillis >= interval) {
    previousMillis += interval;

    // Lee la cantidad actual de pulsos del encoder de forma atómica
    long pulses;
    noInterrupts();
    pulses = encoderPulses;
    interrupts();

    // Calcula la posición en grados y radianes (contando indefinidamente)
    degrees_pos = pulses * (360.0 / 400.0);
    radians_pos = degrees_pos * (PI / 180.0);

    // Calcula el error entre el setpoint y la posición actual
    error = setpoint - degrees_pos;

    // Calcula el control u(n) usando la ecuación de diferencias del PI
    u = prev_u + c1 * error - c2 * prev_error;

    // Limita u al rango [-12V, +12V]
    if (u > 12.0) u = 12.0;
    if (u < -12.0) u = -12.0;

    // Determina la dirección y el valor PWM basado en u
    float dutyCyclePercent = 0.0; // Variable para almacenar el ciclo de trabajo en %

    if (u > 0.0) {
      // Dirección hacia adelante (INA en HIGH, INB en LOW)
      digitalWrite(in1Pin, HIGH);
      digitalWrite(in2Pin, LOW);
      // Mapea u de 0-12V a 0-128 (50% duty cycle)
      int pwmValue = map(u * 100, 0, 1200, 0, 256); // Multiplico por 100 para mayor precisión
      // Calcula el ciclo de trabajo en porcentaje
      dutyCyclePercent = ((float)pwmValue / 256.0) * 100.0; // 128 ≈ 50%
      pwmValue = constrain(pwmValue, 0, 256);
      ledcWrite(pwmChannel, pwmValue);

    }
    else if (u < 0.0) {
      // Dirección inversa (INA en LOW, INB en HIGH)
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, HIGH);
      // Mapea |u| de 0-12V a 0-128 (50% duty cycle)
      int pwmValue = map(-u * 100, 0, 1200, 0, 256); // Multiplico por 100 para mayor precisión
      // Calcula el ciclo de trabajo en porcentaje
      dutyCyclePercent = ((float)pwmValue / 256.0) * 100.0; // 128 ≈ 50%
      pwmValue = constrain(pwmValue, 0, 256);
      ledcWrite(pwmChannel, pwmValue);
    }
    else {
      // Detiene el motor: ambas entradas en HIGH para frenado rápido
      digitalWrite(in1Pin, HIGH);
      digitalWrite(in2Pin, HIGH);
      ledcWrite(pwmChannel, 0);
      dutyCyclePercent = 0.0;
    }

    // Actualiza las variables para el siguiente ciclo
    prev_u = u;
    prev_error = error;

    // Muestra la información relevante por el monitor serial
    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print(" | Pulsos: ");
    Serial.print(pulses);
    Serial.print(" | Posición: ");
    Serial.print(degrees_pos);
    Serial.print(" grados | ");
    Serial.print(radians_pos);
    Serial.print(" radianes | ");
    Serial.print("Control u: ");
    Serial.print(u);
    Serial.print(" V | Duty Cycle: ");
    Serial.print(dutyCyclePercent, 2); // Imprime con 2 decimales
    Serial.println(" %");
  }
}




//Velocidad motor: 1600RPM = 26.6RPS
//Frecuencia máxima del encoder: 26.6RPS * 64CPR = 1706.6Hz
//Esto implica un Tmin entre pulsos de 58.6ms