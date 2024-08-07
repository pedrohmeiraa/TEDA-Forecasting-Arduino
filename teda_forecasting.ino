#include "rls.h"
#include "teda.h"

const int nA = 300;

//Speed
/*
float A[nA] = {6., 13., 18., 23., 28., 33., 37., 42., 44., 43., 41., 39., 32.,
       23., 20., 21., 25., 29., 35., 40., 43., 46., 49., 51., 53., 55.,
       57., 59., 61., 62., 61., 62., 61., 58., 56., 55., 55., 56., 56.,
       56., 56., 55., 46., 33., 19.,  8.,  7., 11., 20., 28., 35., 41.,
       46., 51., 55., 59., 60., 62., 64., 66., 67., 68., 68., 68., 68.,
       69., 68., 67., 66., 66., 68., 68., 68., 68., 67., 65., 63., 61.,
       59., 55., 46., 37., 30., 27., 27., 29., 31., 31., 30., 29., 30.,
       32., 33., 33., 35., 35., 35., 34., 33., 34., 37., 39., 43., 45.,
       49., 52., 55., 56., 59., 61., 63., 63., 64., 63., 63., 64., 64.,
       64., 64., 64., 65., 67., 67., 66., 65., 65., 65., 65., 63., 59.,
       51., 39., 27., 19., 13., 11., 12., 19., 25., 30., 36., 41., 45.,
       49., 51., 52., 49., 46., 45., 44., 45., 46., 48., 51., 54., 58.,
       60., 62., 63., 63., 63., 64., 64., 63., 60., 54., 47., 37., 25.,
       12.,  7., 11., 21., 29., 35., 41., 46., 49., 52., 55., 58., 59.,
       59., 61., 62., 62., 61., 58., 52., 46., 38., 30., 19., 18., 22.,
       28., 33., 37., 41., 43., 43., 39., 33., 30., 33., 37., 42., 44.,
       48., 52., 54., 56., 58., 60., 60., 64., 67., 70., 69., 66., 60.,
       56., 51., 40., 25., 17., 18., 23., 29., 34., 40., 45., 50., 55.,
       58., 59., 58., 57., 57., 57., 56., 53., 50., 47., 43., 40., 33.,
       23., 16., 17., 22., 28., 34., 39., 44., 48., 52., 55., 57., 59.,
       60., 62., 63., 63., 62., 59., 53., 44., 34., 23., 16., 17., 23.,
       30., 36., 42., 47., 51., 53., 55., 58., 60., 63., 65., 68., 69.,
       70., 72., 72., 72., 72., 73., 72., 70., 68., 64., 61., 59., 58.,
       58.};
*/
const int N = 2;
const float lambda = 0.99;
const float delta = 0.1; 

float Y[20] = {};
float Y_virtual[20] = {};
float vetor_de_entrada[N] = {0.0, 0.0};
float x_ant[N] = {0.0, 0.0};
float virtual_input[N] = {0.0, 0.0};
float y_pred = 0;


RLSFilter rls_filter(lambda, delta);
TEDA teda(1.7);
int flag = 0;
int outlier_count = 0;
int N_outlier_max = 3;
bool correction = true;
int forecasting = 2;

const int INTERVALO = 500; // Intervalo de tempo em milissegundos

//Accelerometer variables
float valor_atual=0,y_pred_virtual=0;

void setup() {
  Serial.begin(9600);
  while (!Serial);
//Serial.println("Started");
  Serial.println("CLEARDATA");
  Serial.println("LABEL,Date,Hour,speed,x_ant,x_atual,flag_ar,y_pred_ar,y_pred_virtual, y_virtual_0, y_virtual_1");
  randomSeed(analogRead(0));

}

void loop()
{
    for (int i=0; i < nA; i++)
    {      
      valor_atual = A[i];
      //Serial.print("A[i]: ");
      flag = teda.run(valor_atual);
      if (i==0){
        flag=0;
      }
      //Serial.print("flag agora: ");
      //Serial.print(flag);

      Serial.print("DATA, DATE, TIME,");
      Serial.print(A[i]);
      Serial.print(",");
      Serial.print(x_ant[1]);
      Serial.print(",");
      Serial.print(valor_atual);         


      if (flag == 1){
        outlier_count++;
        valor_atual = y_pred;
      }
      else{
        outlier_count = 0;
      }

      if (correction == true && outlier_count == N_outlier_max + 1)
      {
        valor_atual = A[i];
      }
      
      // Updating weights
      rls_filter.update(valor_atual, x_ant);

      vetor_de_entrada[0] = x_ant[1];
      vetor_de_entrada[1] = valor_atual;
      
      // Predicting
      y_pred = rls_filter.filter(vetor_de_entrada);

      //Virtual filter
      RLSFilter virtual_filter = rls_filter;
      virtual_filter.update(y_pred, vetor_de_entrada);

      virtual_input[0] = vetor_de_entrada[1];
      virtual_input[1] = y_pred;

      for (int j = 0; j < forecasting; j++)
      {
        y_pred_virtual = virtual_filter.filter(virtual_input);
        virtual_filter.update(y_pred_virtual, virtual_input);
        Y_virtual[j] = y_pred_virtual;
  
        virtual_input[0] = virtual_input[1];
        virtual_input[1] = y_pred_virtual;
      }      

      // Second stage
      if (flag == 1){
          valor_atual = y_pred;
      }
      // Consecutive outliers treating
      if(correction == true && outlier_count == N_outlier_max + 1){
          outlier_count = 0;
          valor_atual = A[i];
      }
      
      //LABEL,Date,Hour,3_speed, 4_x_ant, 5_x_atual, 6_flag, 7_y_pred_ar, 8_y_pred_virtual, 9_y_virtual_0, 10_y_virtual_1");
      Serial.print(",");      
      Serial.print(flag);
      Serial.print(",");                         
      Serial.print(y_pred);
      Serial.print(",");
      Serial.print(y_pred_virtual);     
      Serial.print(",");
      Serial.print(Y_virtual[0]);  
      Serial.print(",");
      Serial.println(Y_virtual[1]);  

      x_ant[0] = vetor_de_entrada[0];
      x_ant[1] = vetor_de_entrada[1];

      delay(INTERVALO);  
    }
         
 }
