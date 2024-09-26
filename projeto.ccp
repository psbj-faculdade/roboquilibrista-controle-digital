// Biblioteca Write para comunicação I2C com o MPU6050
#include <Wire.h>

// Biblioteca Servo para controlar os servo motores
#include <Servo.h>

// Nomeando os servos da direita e da esquerda
Servo servoD;
Servo servoE;

#define MPU 0x68   // Endereço do MPU6050

// Variáveis para o acelerômetro
double aceleroX, aceleroY, aceleroZ, somaAceleroX, somaAceleroY, somaAceleroZ;

// Contagem do tempo
double tempoAntes, deltaT;

// Variáveis para o giroscópio
double giroX, giroY, giroZ, somaGiroX, somaGiroY, somaGiroZ, anguloGiroX;

// Variáveis que armazenam os erros
double erroDerivativo, erroProporcional, erroInicial;
double erroIntegral = 0.0;

// Variáveis para as medidas de inclinação
double valorAngulo, anguloInicial, anguloFiltrado = 0.0;

// Constantes do algoritmo PID
double kp = 4.0;
double ki = 50.0;
double kd = 0.2;

// Variável para o valor da correção
int correcao;

// Variável para definir o ponto de equilíbrio
double pontoEquilibrio = 0.0;

void setup()
{
  // Definição dos pinos para os servos e início da comunicação serial
  servoD.attach(9);
  servoE.attach(11);

  // Os servos são configurados inicialmente para ficarem parados
  servoD.write(90);
  servoE.write(90);

  // Função que inicializa o MPU6050
  iniciaMPU();

  // Soma de dez medidas do acelerômetro
  for(int i = 0; i < 10; i++)
  {
    medidasAcelero(6);
    somaAceleroX += aceleroX;
    somaAceleroY += aceleroY;
    somaAceleroZ += aceleroZ;
  }

  // Calcula a média das medidas
  aceleroX = somaAceleroX / 10;
  aceleroY = somaAceleroY / 10;
  aceleroZ = somaAceleroZ / 10;

  // O primeiro ângulo filtrado será o medido pelo acelerômetro
  anguloFiltrado = calculaRoll(aceleroX, aceleroZ, aceleroY);

  // O ângulo inicial do giroscópio será igual ao medido pelo acelerômetro
  anguloGiroX = anguloFiltrado;
}

void loop()
{
  // Zera as somas antes de calcular as médias
  somaGiroX = 0;
  somaGiroY = 0;
  somaGiroZ = 0;
  somaAceleroX = 0;
  somaAceleroY = 0;
  somaAceleroZ = 0;

  // Soma das medidas do acelerômetro e do giroscópio
  for(int i = 0; i < 10; i++)
  {
    medidasAcelero(6);
    medidasGiro(6);
    somaAceleroX += aceleroX;
    somaAceleroY += aceleroY;
    somaAceleroZ += aceleroZ;
    somaGiroX += giroX;
    somaGiroY += giroY;
    somaGiroZ += giroZ;
  }

  // Calcula as médias
  aceleroX = somaAceleroX / 10;
  aceleroY = somaAceleroY / 10;
  aceleroZ = somaAceleroZ / 10;
  giroX = (somaGiroX / 10) + 50.0; // O valor 50.0 é uma correção de desvio nas medidas cruas
  giroY = somaGiroY / 10;
  giroZ = somaGiroZ / 10;

  // Calcula o valor do ângulo pelo acelerômetro (Roll, Pitch e Yaw)
  valorAngulo = calculaRoll(aceleroX, aceleroZ, aceleroY);

  // Intervalo de tempo entre as medidas
  deltaT = (millis() - tempoAntes) / 1000.0;
  tempoAntes = millis();

  // Atualiza o ângulo com base nas medidas do giroscópio
  anguloGiroX += giroX * deltaT / 131.0;

  /*
   * O ângulo filtrado é uma combinação das medições do acelerômetro e do giroscópio,
   * usando um filtro complementar. A soma dos fatores 0.98 e 0.02 deve ser 1.00.
   */
  anguloFiltrado = 0.98 * (anguloFiltrado + (giroX * deltaT / 131.0)) + 0.02 * valorAngulo;

  // Calcula o erro proporcional (diferença entre o ângulo medido e o ponto de equilíbrio)
  erroProporcional = anguloFiltrado - pontoEquilibrio;

  // Calcula a integral do erro ao longo do tempo
  erroIntegral += (erroProporcional + erroInicial) * deltaT / 2.0;

  // Calcula o erro derivativo (taxa de variação do erro)
  erroDerivativo = (erroProporcional - erroInicial) / deltaT;

  // Atualiza o erro inicial para os próximos cálculos
  erroInicial = erroProporcional;

  // Calcula a correção com base nos erros (PID)
  correcao = (int)(kp * erroProporcional + ki * erroIntegral + kd * erroDerivativo);

  // Limita a correção entre -90 e 90, compatível com o controle dos servos
  if(correcao > 90) correcao = 90;
  if(correcao < -90) correcao = -90;

  // Aplica a correção nos servos
  servoD.write(90 + correcao);
  servoE.write(91 - correcao);

  /*
  Serial.print("Giroscópio = ");
  Serial.print(anguloGiroX);
  Serial.print("      Valor angulo = ");
  Serial.print(valorAngulo);
  Serial.print("      Angulo filtrado = ");
  Serial.print(anguloFiltrado);
  Serial.print("      Tempo = ");
  Serial.println(deltaT);
  Serial.print("     Erro deri = ");
  Serial.print(erroDerivativo);
  Serial.print("     Erro prop = ");
  Serial.print(erroProporcional);
  Serial.print("     Erro int = ");
  Serial.print(erroIntegral);
  Serial.print("     Erro = ");
  Serial.println(erro);
  */
}

/*
 * O ângulo medido pelo acelerômetro é calculado
 * usando as medições X, Y e Z dos eixos
 */
double calculaRoll(double A, double B, double C)
{
  double valorA, valorB, resultado;
  valorA = A;
  valorB = sqrt((B * B) + (C * C));
  resultado = atan2(valorA, valorB);
  resultado = resultado * 180 / 3.14;

  return resultado;
}

// Função que inicializa o MPU6050
void iniciaMPU()
{
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);   // Endereço do registro PWR_MGMT_1 para inicializar o MPU6050
  Wire.write(0);      // Tiramos o MPU-6050 do modo sleep
  Wire.endTransmission(true);
  delay(1000);
}

// Função para ler os valores X, Y, Z do acelerômetro
void medidasAcelero(int quantidade)
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3D);  // Inicia a leitura no registro 0x3B
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, quantidade, true);  // Pede 6 bytes ao MPU
  aceleroX = Wire.read() << 8 | Wire.read();  // ACCEL_XOUT_H & ACCEL_XOUT_L
  aceleroY = Wire.read() << 8 | Wire.read();  // ACCEL_YOUT_H & ACCEL_YOUT_L
  aceleroZ = Wire.read() << 8 | Wire.read();  // ACCEL_ZOUT_H & ACCEL_ZOUT_L
}

// Função para ler os valores do giroscópio
void medidasGiro(int quantidade)
{
  Wire.beginTransmission(MPU);
  Wire.write(0x43);  // Inicia a leitura no registro 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, quantidade, true);  // Pede 6 bytes ao MPU
  giroX = Wire.read() << 8 | Wire.read();  // GIRO_XOUT_H & GIRO_XOUT_L
  giroY = Wire.read() << 8 | Wire.read();  // GIRO_YOUT_H & GIRO_YOUT_L
  giroZ = Wire.read() << 8 | Wire.read();  // GIRO_ZOUT_H & GIRO_ZOUT_L
}
