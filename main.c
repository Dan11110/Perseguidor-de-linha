#include <avr/io.h>
#include <util/delay.h>

// Definição dos pinos dos sensores
#define SENSOR_ESQUERDO A0
#define SENSOR_DIREITO A1

// Definição dos pinos dos motores
#define MOTOR_ESQUERDO 3
#define MOTOR_DIREITO 4

// Definição das constantes do PID
#define KP 0.3
#define KI 0.01
#define KD 0.05

// Variáveis para leitura dos sensores
volatile uint8_t leituraEsquerdo;
volatile uint8_t leituraDireito;

// Variável para cálculo do erro
volatile int8_t erro;

// Variáveis para controle PID
float p, i, d, pid;

// Variáveis para controle dos motores
volatile uint8_t velocidadeEsquerdo;
volatile uint8_t velocidadeDireito;

// Função para leitura dos sensores
void leituraSensores() {
  leituraEsquerdo = analogRead(SENSOR_ESQUERDO);
  leituraDireito = analogRead(SENSOR_DIREITO);
}

// Função para cálculo do erro
void calculoErro() {
    if(leituraDireito >= leituraEsquerdo){
        erro = leituraDireito - leituraEsquerdo;
    }else{
        erro = leituraEsquerdo - leituraDireito;
    }
}

// Função para cálculo das ações do PID
void calculoPID() {
  p = erro * KP;
  i += erro * KI;
  d = (erro - erroAnterior) * KD;
  pid = p + i + d;
}

// Função para controle dos motores
void controleMotores() {
  velocidadeEsquerdo = 255 - pid;
  velocidadeDireito = 255 + pid;

  // Limitando as velocidades
  if (velocidadeEsquerdo < 0) {
    velocidadeEsquerdo = 0;
  } else if (velocidadeEsquerdo > 255) {
    velocidadeEsquerdo = 255;
  }
  if (velocidadeDireito < 0) {
    velocidadeDireito = 0;
  } else if (velocidadeDireito > 255) {
    velocidadeDireito = 255;
  }

  // Aplicação das velocidades nos motores
  analogWrite(MOTOR_ESQUERDO, velocidadeEsquerdo);
  analogWrite(MOTOR_DIREITO, velocidadeDireito);
}

// Variável para armazenar o erro anterior
volatile int8_t erroAnterior;

void setup() {
  // Inicialização da serial
  // (descomente se for usar a serial)
  // Serial.begin(9600);

  // Configuração dos pinos dos sensores como entrada
  DDRC &= ~((1 << SENSOR_ESQUERDO) | (1 << SENSOR_DIREITO));

  // Configuração dos pinos dos motores como saída
  DDRD |= ((1 << MOTOR_ESQUERDO) | (1 << MOTOR_DIREITO));

  // Inicialização das variáveis
  leituraEsquerdo = 0;
  leituraDireito = 0;
  erro = 0;
  p = 0;
  i = 0;
  d = 0;
  pid = 0;
  velocidadeEsquerdo = 0;
  velocidadeDireito = 0;
  erroAnterior = 0;
}

void loop() {
  // Leitura dos sensores
  leituraSensores();

  // Cálculo do erro
  calculoErro();

  // Cálculo das ações do PID
  calculoPID();

  // Controle dos motores
  controleMotores();

  // Atualização do erro anterior
  erroAnterior = erro;

  // Delay para evitar sobrecarga da serial
  // (descomente se for usar a serial)
  // delay(10);
}
