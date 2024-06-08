#include <MFRC522.h> //biblioteca responsável pela comunicação com o módulo RFID-RC522
#include <SPI.h> //biblioteca para comunicação do barramento SPI
#include <Nextion.h>
#include <Wire.h>

typedef union{
  uint16_t data;
  struct{
    uint8_t data_array[2];
  };
}data_i2c;

volatile uint8_t stopSignal=0x00;

data_i2c data;

volatile bool stopFlag = false;

// Declaracao das paginas

NexPage p1 = NexPage(0,0,"tela_1");
NexPage p2 = NexPage(1,0,"tela_2");
NexPage p3 = NexPage(2,0,"tela_3");
NexPage p4 = NexPage(3,0,"tela_4");
NexPage p5 = NexPage(4,0,"tela_5");

//botoes

NexButton p1_b3 = NexButton(0,2,"b3");
NexButton p1_b4 = NexButton(0,3,"b4");

NexButton p2_b3 = NexButton(1,2,"b3");
NexButton p2_b4 = NexButton(1,3,"b4");

NexButton p3_b3 = NexButton(2,2,"b3");
NexButton p3_b4 = NexButton(2,3,"b4");

NexButton p4_b3 = NexButton(3,2,"b3");
NexButton p4_b4 = NexButton(3,3,"b4");

NexButton p5_b3 = NexButton(4,2,"b3");
NexButton p5_b4 = NexButton(4,3,"b4");


//declaracao das imagens
NexPicture p1_pic2 = NexPicture(0,9,"p2");
NexPicture p2_pic2 = NexPicture(1,9,"p2");
NexPicture p3_pic2 = NexPicture(2,9,"p2");
NexPicture p4_pic2 = NexPicture(3,9,"p2");
NexPicture p5_pic2 = NexPicture(4,9,"p2");


//declaracao dos numeros

NexNumber p1_n0 = NexNumber(0, 4, "n0");
NexNumber p2_n0 = NexNumber(1, 4, "n0");
NexNumber p3_n0 = NexNumber(2, 4, "n0");
NexNumber p4_n0 = NexNumber(3, 4, "n0");
NexNumber p5_n0 = NexNumber(4, 4, "n0");

//declaracao barras de progresso

NexProgressBar p1_j0 = NexProgressBar(0, 5, "j0"); 
NexProgressBar p2_j0 = NexProgressBar(1, 5, "j0");
NexProgressBar p3_j0 = NexProgressBar(2, 5, "j0");
NexProgressBar p4_j0 = NexProgressBar(3, 5, "j0");
NexProgressBar p5_j0 = NexProgressBar(4, 5, "j0");

//declaracao textos

NexText p1_t0 = NexText(0, 7, "t0"); 
NexText p1_t1 = NexText(0, 8, "t1"); 
NexText p1_t2 = NexText(0, 10, "t2"); 


NexText p2_t0 = NexText(1, 7, "t0");
NexText p2_t1 = NexText(1, 8, "t1"); 
NexText p2_t2 = NexText(1, 10, "t2"); 

NexText p3_t0 = NexText(2, 7, "t0");
NexText p3_t1 = NexText(2, 8, "t1"); 
NexText p3_t2 = NexText(2, 10, "t2"); 

NexText p4_t0 = NexText(3, 7, "t0");
NexText p4_t1 = NexText(3, 8, "t1"); 
NexText p4_t2 = NexText(3, 10, "t2"); 

NexText p5_t0 = NexText(4, 7, "t0");
NexText p5_t1 = NexText(4, 8, "t1"); 
NexText p5_t2 = NexText(4, 10, "t2"); 


#define SS_PIN    27
#define RST_PIN   22


#define SIZE_BUFFER     18
#define MAX_SIZE_BLOCK  16


/*
 * Calibração:
 * 1. Colocar o valor do CALIB_FACTOR em 1.0, como abaixo:
 * #define CALIB_FACTOR 1.0
 * 2. Programe a máquina utilizando o cabo usb.
 * 3. Após programado, retire o cabo usb e deligue e ligue a máquina.
 * 4. Com um copo milimetrado, faça a medição estabelecendo um valor fixo de mLs no copo. Ex:
 * -Fazer 5 tiradas de água de 300 mL cada, anotando os  valores que são dados na tela do sistema.
 * 5. Com os valores anotados, faça uma média aritmética dos valores (MED) e, por fim, obtenha o valor final de CALIB_FACTOR na forma:
 *    CALIB_FACTOR = 300mL/MED
 * Lembre-se que o valor de 300 mL pode ser diferente dependendo da quantidade que você estabeleceu no copo milimetrado. 
 * 6. Troque agora o valor de CALIB_FACTOR, pelo novo valor calibrado.
 * 7. Conecte o cabo usb, e realize a programação.
 * 8. Desligue e ligue o sistema.
 * 9. Verifique se o valor dos mLs está correto.
 * 10. Repita todos os passos se a calibração ainda não estiver correta.
 */


#define CALIB_FACTOR  2.9761   //Aqui definimos a calibragem







//esse objeto 'chave' é utilizado para autenticação
MFRC522::MIFARE_Key key;
//código de status de retorno da autenticação
MFRC522::StatusCode status;

// Definicoes pino modulo RC522
MFRC522 mfrc522(SS_PIN, RST_PIN);

volatile uint32_t  idCard= 0x00;
volatile uint8_t valvSel = 0;
volatile bool activeMode = false;

float saldoTotal=120.00;
float saldoInicial = 0.0;
float consumption = 0.0;
uint16_t mlCounter = 0;
uint16_t mlCounter_old = 0;
char buf[50];

float choppPrice1 = 0.059;
float choppPrice2 = 0.059;
float choppPrice3 = 0.059;
float choppPrice4 = 0.059;
float choppPrice5 = 0.059;

void valveBoardCom();
void valveBoardClean();
void leituraDados();
void stopFlagAnalyse();



NexTouch *nex_listen_list[] = {
  &p1_b3, &p1_b4, &p2_b3, &p2_b4,&p3_b3, &p3_b4, &p4_b3, &p4_b4, &p5_b3, &p5_b4, NULL
};



void p1_b3_Release(void *ptr) {
  if(activeMode == false){
    idCard = 0x00;
    valvSel = 1;
    p2.show();
    sprintf(buf, "%.3f", choppPrice2);
    p2_t2.setText(buf); 
  }
}

void p1_b4_Release(void *ptr) {
  if(activeMode == false){
    idCard = 0x00;
    valvSel = 4;
    p5.show();
    sprintf(buf, "%.3f", choppPrice5);
    p5_t2.setText(buf); 
  }
}

void p2_b3_Release(void *ptr) {
  if(activeMode == false){
    idCard = 0x00;
    valvSel = 2;
    p3.show();
    sprintf(buf, "%.3f", choppPrice3);
    p3_t2.setText(buf);
  }
}

void p2_b4_Release(void *ptr) {
  if(activeMode == false){
    idCard = 0x00;
    valvSel = 0;
    p1.show();
    sprintf(buf, "%.3f", choppPrice1);
    p1_t2.setText(buf);
      
  }
  
}


void p3_b3_Release(void *ptr) {
  if(activeMode == false){
    idCard = 0x00;
    valvSel = 3;
    p4.show();
    sprintf(buf, "%.3f", choppPrice4);
    p4_t2.setText(buf);
  }
}

void p3_b4_Release(void *ptr) {
  if(activeMode == false){
    idCard = 0x00;
    valvSel = 1;
    p2.show();
    sprintf(buf, "%.3f", choppPrice2);
    p2_t2.setText(buf);
      
  }
  
}


void p4_b3_Release(void *ptr) {
  if(activeMode == false){
    idCard = 0x00;
    valvSel = 4;
    p5.show();
    sprintf(buf, "%.3f", choppPrice5);
    p5_t2.setText(buf);
  }
}

void p4_b4_Release(void *ptr) {
  if(activeMode == false){
    idCard = 0x00;
    valvSel = 2;
    p3.show();
    sprintf(buf, "%.3f", choppPrice3);
    p3_t2.setText(buf);
      
  }
  
}


void p5_b3_Release(void *ptr) {
  if(activeMode == false){
    idCard = 0x00;
    valvSel = 0;
    p1.show();
    sprintf(buf, "%.3f", choppPrice1);
    p1_t2.setText(buf);
  }
}

void p5_b4_Release(void *ptr) {
  if(activeMode == false){
    idCard = 0x00;
    valvSel = 3;
    p4.show();
    sprintf(buf, "%.3f", choppPrice4);
    p4_t2.setText(buf);
      
  }
  
}


void setup() {
  // Inicia a serial
  Serial.begin(115200);
  SPI.begin(); // Init SPI bus

  Wire.begin();

  
  valveBoardClean();

  
  // Inicia MFRC522
  mfrc522.PCD_Init(); 
  // Mensagens iniciais no serial monitor
  nexInit();

  //atribuicao dos botoes
  
  p1_b3.attachPop(p1_b3_Release, &p1_b3);
  p1_b4.attachPop(p1_b4_Release, &p1_b4);
  
  p2_b3.attachPop(p2_b3_Release, &p2_b3);
  p2_b4.attachPop(p2_b4_Release, &p2_b4);

  p3_b3.attachPop(p3_b3_Release, &p3_b3);
  p3_b4.attachPop(p3_b4_Release, &p3_b4);

  p4_b3.attachPop(p4_b3_Release, &p4_b3);
  p4_b4.attachPop(p4_b4_Release, &p4_b4);

  p5_b3.attachPop(p5_b3_Release, &p5_b3);
  p5_b4.attachPop(p5_b4_Release, &p5_b4);


//inicializacao com o a primeira tela

  p1.show();
  sprintf(buf, "%.3f", choppPrice1);
  p1_t2.setText(buf);

}

void loop() {
  nexLoop(nex_listen_list);

// Aguarda a aproximacao do cartao

  if (mfrc522.PICC_IsNewCardPresent()){
    // Seleciona um dos cartoes
    if (mfrc522.PICC_ReadCardSerial()){
      leituraDados();
    }
  }


  if((idCard == 0xC692C42B) && (activeMode == false)){
    saldoTotal = 120.00;
    saldoInicial = saldoTotal;
    idCard = 0x00;
    stopFlag = false;

    sprintf(buf, "%.2f", saldoTotal);
    if(valvSel == 0){
        p1_pic2.setPic(9);
        p1_t0.setText("0.00");
        p1_t1.setText(buf);    //Valores ainda ficticios;
        valveBoardInit();    
    }
    
    else if (valvSel == 1){
        p2_pic2.setPic(9);     
        p2_t0.setText("0.00");
        p2_t1.setText(buf);    //Valores ainda ficticios;
        valveBoardInit();    

    }
    else if (valvSel == 2){
        p3_pic2.setPic(9);     
        p3_t0.setText("0.00");
        p3_t1.setText(buf);    //Valores ainda ficticios;
        valveBoardInit();    

    }
    else if (valvSel == 3){
        p4_pic2.setPic(9);     
        p4_t0.setText("0.00");
        p4_t1.setText(buf);    //Valores ainda ficticios;
        valveBoardInit();    

    }
    else if (valvSel == 4){
        p5_pic2.setPic(9);     
        p5_t0.setText("0.00");
        p5_t1.setText(buf);    //Valores ainda ficticios;
        valveBoardInit();    

    }
    
  }
  else if(activeMode == true && stopFlag == false){
      valveBoardCom();    
      if(valvSel == 0){
        mlCounter = (uint16_t)(data.data*CALIB_FACTOR);
        //Serial.println(data.data);
        consumption = (float)(mlCounter*choppPrice1);
        saldoTotal = saldoInicial - consumption;
        sprintf(buf, "%.2f", consumption);
        p1_t0.setText(buf);
        sprintf(buf, "%.2f", saldoTotal);
        p1_t1.setText(buf);
        p1_n0.setValue(mlCounter);
        p1_j0.setValue((uint32_t)(mlCounter/20));
  
        if(mlCounter >= 2000 || saldoTotal <=0){
          activeMode = false;
          idCard = 0x00;
          valveBoardClean();
          p1.show();
      }
    }
    else if(valvSel == 1){
      valveBoardCom();
      mlCounter = (uint16_t)(data.data*CALIB_FACTOR);
      //Serial.println(data.data);
      consumption = (float)(mlCounter*choppPrice2);
      saldoTotal = (float)(saldoInicial - consumption);
      sprintf(buf, "%.2f", consumption);
      p2_t0.setText(buf);
      sprintf(buf, "%.2f", saldoTotal);
      p2_t1.setText(buf);
      p2_n0.setValue(mlCounter);
      p2_j0.setValue((uint32_t)(mlCounter/20));

      if(mlCounter >= 2000 || saldoTotal <=0){
        activeMode = false;
        idCard = 0x00;
        valveBoardClean();
        p2.show();
      }
    }
    else if(valvSel == 2){
      valveBoardCom();
      mlCounter = (uint16_t)(data.data*CALIB_FACTOR);
      //Serial.println(data.data);
      consumption = (float)(mlCounter*choppPrice3);
      saldoTotal = (float)(saldoInicial - consumption);
      sprintf(buf, "%.2f", consumption);
      p3_t0.setText(buf);
      sprintf(buf, "%.2f", saldoTotal);
      p3_t1.setText(buf);
      p3_n0.setValue(mlCounter);
      p3_j0.setValue((uint32_t)(mlCounter/20));

      if(mlCounter >= 2000 || saldoTotal <=0){
        activeMode = false;
        idCard = 0x00;
        valveBoardClean();
        p3.show();

      }
    } 
    else if(valvSel == 3){
      valveBoardCom();
      mlCounter = (uint16_t)(data.data*CALIB_FACTOR);
      //Serial.println(data.data);
      consumption = (float)(mlCounter*choppPrice4);
      saldoTotal = (float)(saldoInicial - consumption);
      sprintf(buf, "%.2f", consumption);
      p4_t0.setText(buf);
      sprintf(buf, "%.2f", saldoTotal);
      p4_t1.setText(buf);
      p4_n0.setValue(mlCounter);
      p4_j0.setValue((uint32_t)(mlCounter/20));

      if(mlCounter >= 2000 || saldoTotal <=0){
        activeMode = false;
        idCard = 0x00;
        valveBoardClean();
        p4.show();

      }
    } 
    else if(valvSel == 4){
      valveBoardCom();
      mlCounter = (uint16_t)(data.data*CALIB_FACTOR);
      //Serial.println(data.data);
      consumption = (float)(mlCounter*choppPrice5);
      saldoTotal = (float)(saldoInicial - consumption);
      sprintf(buf, "%.2f", consumption);
      p5_t0.setText(buf);
      sprintf(buf, "%.2f", saldoTotal);
      p5_t1.setText(buf);
      p5_n0.setValue(mlCounter);
      p5_j0.setValue((uint32_t)(mlCounter/20));

      if(mlCounter >= 2000 || saldoTotal <=0){
        activeMode = false;
        idCard = 0x00;
        valveBoardClean();
        p5.show();

      }
    }     
  }
    else if(stopFlag == true){
      //Serial.println("Flag de parada recebida");
      activeMode = false;
      stopSignal = 0x00;
      mlCounter = 0;
      consumption = 0;
      saldoTotal = 0;
    
    
      if(valvSel == 0){
        idCard = 0x00;
        valveBoardClean();
        p1.show();
        sprintf(buf, "%.3f", choppPrice1);
        p1_t2.setText(buf);
      }
      else if(valvSel == 1){
        idCard = 0x00;
        valveBoardClean();
        p2.show();
        sprintf(buf, "%.3f", choppPrice2);
        p2_t2.setText(buf);
      }
      else if(valvSel == 2){
        idCard = 0x00;
        valveBoardClean();
        p3.show();
        sprintf(buf, "%.3f", choppPrice3);
        p3_t2.setText(buf);
      }
      else if(valvSel == 3){
        idCard = 0x00;
        valveBoardClean();
        p4.show();
        sprintf(buf, "%.3f", choppPrice4);
        p4_t2.setText(buf);
      }
      else if(valvSel == 4){
        idCard = 0x00;
        valveBoardClean();
        p5.show();
        sprintf(buf, "%.3f", choppPrice5);
        p5_t2.setText(buf);
      }
  }

  
  else{
    activeMode = false;
    stopFlag = false;
  }


  stopFlagAnalyse();


  mfrc522.PICC_HaltA(); 
  mfrc522.PCD_StopCrypto1(); 




}

void stopFlagAnalyse(){

  if (stopSignal == 0x00){
    stopFlag = false;
  }
  else if (stopSignal == 0xff){
    stopFlag = true;
  }

}

void valveBoardInit(){
  Wire.beginTransmission(0x08);
  Wire.write(0b11000000 | (0x01 << valvSel));
  Wire.endTransmission();

  delay(500);

  Wire.beginTransmission(0x08); 
  Wire.write(0b00000000);
  Wire.endTransmission();


  Wire.beginTransmission(0x08);
  Wire.write((0b01000000) | valvSel);
  Wire.endTransmission();

  activeMode = true;

}

void valveBoardCom(){

  Wire.beginTransmission(0x08);
  Wire.write(0b11000000 | (0x01 << valvSel));
  Wire.endTransmission();
  
  Wire.beginTransmission(0x08);
  Wire.write((0b01000000) | valvSel);
  Wire.endTransmission();


  
  Wire.requestFrom(0x08, 3,1);
  delay(1);

  while (!Wire.available()); 
    data.data_array[0]= Wire.read();

    data.data_array[1]= Wire.read(); 

    stopSignal = Wire.read();  
    //Serial.print("valor do stopSignal:");
    //Serial.println(stopSignal,HEX); 
}


void valveBoardClean(){
  Wire.beginTransmission(0x08);
  Wire.write(0b10000000);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x08);
  Wire.write(0b00000000);
  Wire.endTransmission();
}



void leituraDados()
{
  //imprime os detalhes tecnicos do cartão/tag
  Serial.println("dados dentro"); 
  mfrc522.PICC_DumpDetailsToSerial(&(mfrc522.uid)); 

  idCard = mfrc522.uid.uidByte[0] << 24;
  idCard += mfrc522.uid.uidByte[1] << 16;
  idCard += mfrc522.uid.uidByte[2] << 8;
  idCard += mfrc522.uid.uidByte[3]; 


}
