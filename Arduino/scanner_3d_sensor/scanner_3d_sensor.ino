/*
 * Scanner 3D - Sensor HC-SR04
 * 
 * Lê continuamente o sensor ultrassônico HC-SR04 a 10Hz
 * e envia as leituras via Serial para o Python processar.
 * 
 * Conexões:
 * HC-SR04 VCC -> Arduino 5V
 * HC-SR04 GND -> Arduino GND
 * HC-SR04 TRIG -> Arduino Pin 11
 * HC-SR04 ECHO -> Arduino Pin 10
 */

// Pinos do sensor HC-SR04
const int TRIG_PIN = 11;
const int ECHO_PIN = 10;

// Configurações
const int SAMPLE_RATE = 10; // Hz - 10 leituras por segundo
const int SAMPLE_INTERVAL = 1000 / SAMPLE_RATE; // ms

// CALIBRAÇÃO - Ajuste este valor até a medida ficar correta
// Padrão: 0.0343 (som a 20°C)
// Se medir MAIOR que real: DIMINUA o valor (ex: 0.0330)
// Se medir MENOR que real: AUMENTE o valor (ex: 0.0355)
const float SOUND_SPEED = 0.0343; // cm/µs

// Variáveis
unsigned long lastSampleTime = 0;

void setup() {
  // Inicializa comunicação serial a 115200 baud
  Serial.begin(115200);
  
  // Configura pinos do sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Aguarda estabilização
  delay(100);
  
  // Sinal de inicialização
  Serial.println("READY");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Amostragem a 10Hz
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = currentTime;
    
    // Pulso de trigger
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Lê duração
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    
    if (duration == 0) {
      // Envia erro no formato que o Python entende
      Serial.println("DIST:ERROR");
    } else {
      float distance = (duration * SOUND_SPEED) / 2.0;
      
      // Formato limpo para o Python
      Serial.print("DIST:");
      Serial.println(distance, 2);
    }
  }
}

/**
 * Lê a distância do sensor HC-SR04
 * Retorna a distância em centímetros
 */
float readDistance() {
  // Pulso de trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Lê o tempo do pulso echo
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout de 30ms
  
  // Calcula distância em cm
  // Velocidade do som = 343 m/s = 0.0343 cm/µs
  // Distância = (tempo * velocidade) / 2
  if (duration == 0) {
    return -1; // Timeout
  }
  
  float distance = (duration * 0.0343) / 2.0;
  
  return distance;
}
