/*
 * TESTE SIMPLES HC-SR04
 * Use este código para verificar se o sensor está funcionando
 */

const int TRIG_PIN = 11;
const int ECHO_PIN = 10;

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  Serial.println("=== TESTE HC-SR04 ===");
  Serial.println("Pinos: TRIG=11, ECHO=10");
  Serial.println();
  delay(1000);
}

void loop() {
  // Pulso de trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Lê duração do pulso
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  // Debug completo
  Serial.print("Duration: ");
  Serial.print(duration);
  Serial.print(" us | ");
  
  if (duration == 0) {
    Serial.println("TIMEOUT - Sensor não responde!");
  } else {
    float distance = (duration * 0.0343) / 2.0;
    Serial.print("Distancia: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
  
  delay(1000); // 1 segundo entre leituras
}
