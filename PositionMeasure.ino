// Pines a los que está conectado el encoder
const int encoderPinA = 36;  // Pin A del encoder (GPIO 36)
const int encoderPinB = 25;  // Pin B del encoder (GPIO 25)

// Pines de control del driver H
const int enablePin = 0;     // Cambiado a GPIO 16
const int in1Pin = 2;         // Pin IN1 del driver H (GPIO 2)
const int in2Pin = 14;        // Pin IN2 del driver H (GPIO 14)

// Pin del potenciómetro
const int potPin = 39;        // Pin del potenciómetro (GPIO 39)

float degrees = 0;
float radians = 0;

// Variables para el conteo de pulsos y dirección
volatile long encoderPulses = 0;       // Contador de pulsos
volatile int lastEncoderAState = LOW;  // Último estado de la señal A
volatile int lastEncoderBState = LOW;  // Último estado de la señal B

// Configuración LEDC
const int pwmChannel = 0;
const int pwmFrequency = 5000; // 5 kHz
const int pwmResolution = 8;   // 8 bits

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

void setup() {
  // Inicializa enablePin como OUTPUT y establece LOW inmediatamente
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW); // Asegura que el pin esté LOW antes de configurar LEDC

  // Configura LEDC
  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(enablePin, pwmChannel);

  // Configura los pines del encoder como entradas
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  // Configura los pines del driver H
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);

  // Configuración inicial del motor
  digitalWrite(in1Pin, LOW);     // Configura IN1 en LOW
  digitalWrite(in2Pin, HIGH);    // Configura IN2 en HIGH para definir el sentido

  // Inicializa la comunicación serial
  Serial.begin(115200);

  // Adjunta interrupciones a los canales A y B del encoder
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), encoderBISR, CHANGE);
}

void loop() {
  // Lee el valor del potenciómetro y ajusta la velocidad del motor
  int potValue = analogRead(potPin);                       // Lee el valor analógico del potenciómetro (0-4095)
  int pwmValue = map(potValue, 0, 4095, 0, 255);           // Convierte a rango de PWM (0-255)

  ledcWrite(pwmChannel, pwmValue);                          // Ajusta el PWM usando LEDC

  // Calcula la posición rotacional en grados
  degrees = (encoderPulses % 400) * (360.0 / 400);

  // Calcula la posición rotacional en radianes
  radians = degrees * (PI / 180.0);

  // Muestra el número de pulsos, el ángulo en grados y en radianes
  Serial.print("Pulsos: ");
  Serial.print(encoderPulses);
  Serial.print(" | Posición: ");
  Serial.print(degrees);
  Serial.print(" grados | ");
  Serial.print(radians);
  Serial.print(" radianes | ");
  Serial.print("Velocidad PWM: ");
  Serial.println(potValue);

  // Pequeño retraso para evitar lecturas demasiado rápidas
  delay(100);
}

