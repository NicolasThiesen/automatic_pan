/* ============================================================== =

  UNO - leitura dos 6 AD simples



    ================================================================== */
#include <string.h>
#include <math.h>

#include <Wire.h> //necessária para a interface i2c do display
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>

LiquidCrystal_I2C lcd(0x3F, 20, 4);



int num_tot_canais = 1;
boolean plotter = true;
int T_Amb_Max = 30 , histerese_tempo = 5, T_Saida_AC;

double a[15], b[15], c[15], d[15], e[15], f[15], g[15];
double V_temp;


double Setpoint, Input, Output;
double Kp=10, Ki=0.02, Kd=0.001;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int rele_pwm = 10, chave_potencia = 8, chave_set=9, V_potencia, V_set;
double pot_temp = 0, pot_w = 0, T_AC1=0,soma_temp = 0;

const float Resolucao = 1024.0;
const float V_ref = 5.0;


//==============================================================================================================================================
//==============================================================================================================================================

void setup()
{
  Serial.begin(9600);

  pinMode(rele_pwm, OUTPUT); // Rele PWM
  pinMode(chave_potencia, INPUT); // Chave de Potencia
  pinMode(chave_set, INPUT); // Chave ?? 
  pinMode(6, OUTPUT); // ???
  pinMode(7, OUTPUT); // ???
  pinMode(10, INPUT); // ???


  a[1] = 0.10900668;  b[1] = -1.10025500;  c[1] = 5.28794594;  d[1] = -14.75771141;  e[1] = 40.30831163;  f[1] = -33.78559142 ;
  a[2] = 0.10900668;  b[2] = -1.10025500;  c[2] = 5.28794594;  d[2] = -14.75771141;  e[2] = 40.30831163;  f[2] = -33.78559142 ;
  a[3] = 0.10900668;  b[3] = -1.10025500;  c[3] = 5.28794594;  d[3] = -14.75771141;  e[3] = 40.30831163;  f[3] = -33.78559142 ;
  a[4] = 0.10900668;  b[4] = -1.10025500;  c[4] = 5.28794594;  d[4] = -14.75771141;  e[4] = 40.30831163;  f[4] = -33.78559142 ;
  //a[9] = 0.0; b[9] = 0.0; c[9] = 0.0; d[9] = 0.0; e[9]= 31.446541;  f[9] = -23.82075; // UR

  // -------------- LCD --------------------
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  // -------------- PID --------------------
  Input = T_AC1;
  Setpoint = pot_temp*105;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  Serial.println("Temperatura_AC Potencia Output");
}

//================================================================================================================================================
//================================================================================================================================================

void loop()
{
  

   for (int i = 1; i <= 100; i++)  
   {
        V_temp =  analogRead(A0)/ Resolucao * V_ref;
        soma_temp += V_temp;
   }
   
   V_temp = soma_temp/100.0;
   soma_temp = 0;
   
   pot_temp = 1-(analogRead(A1)/Resolucao);
   pot_w =  255.0-(analogRead(A2)*255.0/Resolucao);
   Setpoint = pot_temp*105; //varia a temperatura de 0 graus á 105 graus    
   T_AC1 = Calibrar_tensao(V_temp, 1) ;
   V_potencia = digitalRead(chave_potencia);
   V_set = digitalRead(chave_set);
 

if(V_temp > 0.0){ 
   
   if(V_potencia == HIGH && V_set == LOW) {
    analogWrite(rele_pwm, pot_w);
  
  }

   if(V_potencia == LOW && V_set == HIGH){
    
    Input = T_AC1;
    myPID.Compute();
    analogWrite(rele_pwm, Output);
  }
 
 if(V_potencia == HIGH && V_set == HIGH){
    analogWrite(rele_pwm, 0);
  }

if(V_potencia == LOW && V_set == LOW){
    analogWrite(rele_pwm, 0);
  }


}

if(V_temp == 0.0) analogWrite(rele_pwm, 0);

 
  Serial_Print();
  Escreve_no_LCD();
  delay(500);

}

// ========================================================================================================================================
   void Serial_Print()
   {
    
    if(plotter){
    
    Serial.print(T_AC1);
    Serial.print(" ");
    Serial.print(pot_w);
    Serial.print(" ");
    Serial.println(Output);    
  
  }

   else {
   Serial.print("Saida PID: ");
   Serial.print(Output);
   Serial.println("");
   Serial.print("Temperatura T_AC1: ");
   Serial.print(T_AC1);
   Serial.println("");
   Serial.print("V temp: ");
   Serial.print(V_temp);
   Serial.println("");
   Serial.print("Chave V_potencia: ");
   Serial.print(V_potencia);
   Serial.println("");
   Serial.print("Chave V_set: ");
   Serial.println(V_set);
   Serial.print("var pot_w:");
   Serial.println(pot_w);
   Serial.println("========================================================");

   }


    
   }

  
// ========================================================================================================================================

void Escreve_no_LCD()
{


  if( V_potencia == HIGH && V_set == LOW) {
 
  lcd.setCursor(1, 0);    lcd.print("Modo Potencia ON");
    
  }

  if(V_potencia == LOW && V_set == HIGH) {
 
  lcd.setCursor(1, 0);    lcd.print("Modo Setpoint ON");
    
  }

  if(V_potencia == HIGH && V_set == HIGH) {
 
  lcd.setCursor(1, 0);    lcd.print(" Resistencia OFF");
    
  }
if(V_temp == 0.0){
  lcd.setCursor(1, 0);    lcd.print("Sensor Temp descon");
  }

  lcd.setCursor(0, 1);    lcd.print("Temp Mosto:");
  lcd.setCursor(11, 1);   lcd.print(T_AC1, 1);
  lcd.setCursor(15, 1);   lcd.print((char) 223);  lcd.print("C");// degree symbol


  lcd.setCursor(0, 2);    lcd.print("Temp SetPoint:");
  lcd.setCursor(14, 2);   lcd.print(Setpoint, 1);
  lcd.setCursor(18, 2);   lcd.print((char) 223);  lcd.print("C");// degree symbol


  lcd.setCursor(0, 3);    lcd.print("Potencia:");
  lcd.setCursor(9, 3);   lcd.print(pot_w*100/255, 1);
  lcd.setCursor(13, 3);   lcd.print("%");// degree symbol
  
}

// ========================================================================================================================================

double Calibrar_tensao(double tensao, int canal)
{
  double Valor_calibrado;
  Valor_calibrado =    a[canal] * pow(tensao, 5) +
                       b[canal] * pow(tensao, 4) +
                       c[canal] * pow(tensao, 3) +
                       d[canal] * pow(tensao, 2) +
                       e[canal] * pow(tensao, 1) +
                       f[canal] * pow(tensao, 0);
  return Valor_calibrado;
}

//======================================================================================================
