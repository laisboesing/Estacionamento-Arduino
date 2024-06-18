#include <Servo.h>

#define SERVO_PIN 10
#define TRIGGER_PIN1 A0
#define ECHO_PIN1 A1
#define RED_LED 2
#define YELLOW_LED 3
#define GREEN_LED 4
#define BUTTON_PIN 11

Servo servo;

bool cancelaAberta = false;
bool paused = false;
bool buttonState = false;
bool lastButtonState = false;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;  // Delay para debounce
unsigned long lastPauseTime = 0;
unsigned long pauseDuration = 3000;  // Tempo de pausa em milissegundos

// Estados do semáforo
enum TrafficLightState {
  RED,
  GREEN,
  YELLOW
};

TrafficLightState currentState = RED;  // Estado inicial do semáforo

void setup() {
  Serial.begin(9600);

  // Setup do Servo
  pinMode(SERVO_PIN, OUTPUT);
  servo.attach(SERVO_PIN);
  pinMode(TRIGGER_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);

  // Setup dos LEDs e Botão
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
  // Leitura e controle do sensor ultrassônico e do servo
  float distancia1 = medirDistancia(TRIGGER_PIN1, ECHO_PIN1);
  if (distancia1 >= 2 && distancia1 <= 4) {
    if (!cancelaAberta) {
      abrirCancela();
      cancelaAberta = true;
    }
  } else {
    if (cancelaAberta) {
      fecharCancela();
      cancelaAberta = false;
    }
  }

  // Leitura do botão e controle dos LEDs
  int reading = digitalRead(BUTTON_PIN);

  // Verifica se o estado do botão mudou
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  // Se o tempo de debounce tiver passado, considera a leitura atual
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // Se o estado do botão mudou, altera o estado de pausa
    if (reading != buttonState) {
      buttonState = reading;
      // Apenas alterna o estado de pausa quando o botão é pressionado (LOW)
      if (buttonState == LOW) {
        // Mudar para o estado vermelho imediatamente
        currentState = RED;
        lastPauseTime = millis(); // Inicia contagem para retorno ao ciclo normal
      }
    }
  }

  lastButtonState = reading;

  // Controle do semáforo
  switch (currentState) {
    case RED:
      digitalWrite(RED_LED, HIGH);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(GREEN_LED, LOW);
      if (!paused && millis() - lastPauseTime > pauseDuration) {
        currentState = GREEN;
        lastPauseTime = millis();
      }
      break;
    case GREEN:
      digitalWrite(RED_LED, LOW);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(GREEN_LED, HIGH);
      if (!paused && millis() - lastPauseTime > pauseDuration) {
        currentState = YELLOW;
        lastPauseTime = millis();
      }
      break;
    case YELLOW:
      digitalWrite(RED_LED, LOW);
      digitalWrite(YELLOW_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);
      if (!paused && millis() - lastPauseTime > pauseDuration) {
        currentState = RED;
        lastPauseTime = millis();
      }
      break;
  }
}

float medirDistancia(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  return pulseIn(echoPin, HIGH) / 58.0;
}

void abrirCancela() {
  servo.write(90);
  delay(2000);
}

void fecharCancela() {
  servo.write(0);
  delay(2000);
}
