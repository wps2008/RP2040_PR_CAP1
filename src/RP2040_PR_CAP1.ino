#include "pico/stdlib.h"
//#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include "pico/stdlib.h"
#include "rp2040_pr_cap.pio.h"

//GP00-07 DB0-7
//GP08 RS232C TX
//GP09 DBp(O)
//GP10 BSY(O)
//GP11 MSG(O)
//GP12 C/D(O)
//GP13 REQ(O)
//GP14 I/O(O)
//GP15 RST(IO)
//GP16 GP17 GP18 GP19 SDカード
//GP20 I2s 
//GP21 I2s
//GP22 I2s
//GP26 ATN(I)
//GP27 ACK(I)
//GP28 SEL(I)

//RP2040

#define PIN_SD_MOSI       PIN_SPI0_MOSI
#define PIN_SD_MISO       PIN_SPI0_MISO
#define PIN_SD_SCK        PIN_SPI0_SCK
#define PIN_SD_SS         PIN_SPI0_SS

#define GPIO_IN *(volatile uint32_t*)0xD0000004

#define GPIO_OUT *(volatile uint32_t*)0xD0000010
#define GPIO_OUT_SET *(volatile uint32_t*)0xD0000014
#define GPIO_OUT_CLR *(volatile uint32_t*)0xD0000018

#define GPIO_OE *(volatile uint32_t*)0xD0000020
#define GPIO_OE_SET *(volatile uint32_t*)0xD0000024
#define GPIO_OE_CLR *(volatile uint32_t*)0xD0000028

//バス操作ルーチン一覧
#define SEL ((GPIO_IN >> 28) & 1) //GP28

#define io0() GPIO_OUT_CLR = 0x4000; //GP14
#define io1() GPIO_OUT_SET = 0x4000;

#define ledon()  GPIO_OUT_SET = 0x02000000; //GP25
#define ledoff() GPIO_OUT_CLR = 0x02000000;

#define dbus_read()\
  dbus = (uint16_t)((GPIO_IN) & 0x00FF);\   

#define DEBUG Serial

//TF
#define SPI_CLOCK SD_SCK_MHZ(12)
#define SD_CONFIG SdSpiConfig(PIN_SPI0_SS, SHARED_SPI, SPI_CLOCK ,&SPI)
SdFs SD;
FsFile file;

uint16_t dbus;
uint8_t buf[0x10000];
uint16_t buf_p = 0;
uint16_t buf_pa = 0;  
uint16_t buf_pf = 0;
uint8_t readyf = 0;
uint8_t intf = 0;

char message[0x80];

PIO pio = pio0;
uint offset;
uint sm;

//割り込みルーチン
void irq_call0(uint gpio, uint32_t event){

//if (gpio == 28){
    //intf = 1;
    //asm("cpsid if");
    GPIO_OUT_SET = 0x400;  
    dbus_read();
    if (readyf == 0){return;}
    buf[buf_p] = dbus & 0xFF;
    buf_p++;
    GPIO_OUT_CLR = 0x400;  
    //asm("cpsie if");
    //intf = 0;
//}

  }



 void setup1(){
  
  while (readyf == 0){asm("NOP");}
  
  offset = pio_add_program(pio, &pr_cap_program);
  sm = pio_claim_unused_sm(pio, true);
  pr_cap_program_init(pio, sm, offset, 0);
  
  }

 void loop1(){

//while(1){asm("NOP");}
  int res;
  ledon();

  while (1){
    //pio_sm_put_blocking(pio, sm);
   res = pio_sm_get_blocking(pio, sm);
   asm("NOP");
   //DEBUG.println((res & 0xFF),HEX);
   buf[(buf_p % sizeof buf)] = (res & 0xFF);
   buf_p++;
   asm("NOP");
   //int res = pio_sm_get_blocking(pio, sm);
   //DEBUG.println((res & 0xFF),HEX);
    }

}




void setup() {
//シリアルモニタ
Serial.begin(115200);
delay(1000);

//GPIO初期化
 for (int i = 0 ; i < 30 ; i++ )
 {
  if (i == 8 || i == 16 || i == 17 || i == 18 || i == 19){continue;} 
  gpio_init(i);
 }

 gpio_set_dir(28, true);
 digitalWrite(28,0);
 gpio_set_dir(28, false);

//プルアップ
 gpio_pull_up(0);
 gpio_pull_up(1);
 gpio_pull_up(2);
 gpio_pull_up(3);
 gpio_pull_up(4);
 gpio_pull_up(5);
 gpio_pull_up(6);
 gpio_pull_up(7);        
 //gpio_pull_up(28);

 GPIO_OE = 0b00000011110000000111111000000000;

// digitalWrite(10,1);
 
 /*
 GPIO_OE_SET = 0x000002FF; 
 GPIO_OUT &= 0xFFFFFD00;
 delay(10);
 */
 GPIO_OE_CLR = 0x000002FF;

 io1();
 ledoff();

//goto GPIO_INT;

//TFカード初期化
  if (!SD.begin(SD_CONFIG))
  {
    DEBUG.println("TF_Card Initialization failed!");
    led_fail(500);
  }

//カード内のディレクトリ/pri_captを開き(ない場合は自動作成)
//0000.binから順番に探して存在しないファイルに到達したら書き込む。
//リセットされるまで同一ファイルに書き込まれる。
char fname[512];
if (!SD.exists("/pri_capt")){SD.mkdir("/pri_capt");}
for (int i = 0; i < 65531; i++){
  sprintf (fname,"/pri_capt/%04u.bin",i);
  if (!SD.exists(fname)){break;}
  if (i == 65530){led_fail(1500);}
  }


file = SD.open(fname, FILE_WRITE);
if (file == false){led_fail(2000);}


GPIO_INT:
//最後に割り込みをセットする
//gpio_set_irq_enabled_with_callback(28, GPIO_IRQ_EDGE_FALL, 1 ,irq_call0 );

  for (int i = 0; i < 20; i++){
     digitalWrite(12,0);
     for (int ii = 0; ii < 20; ii++){asm("NOP");}
     digitalWrite(12,1);
     for (int ii = 0; ii < 20; ii++){asm("NOP");}
    }


//準備完了
DEBUG.println("RP2040 Printer Capture Ready");
//ledon();
readyf = 1;

}

 //メインループ
void loop() {
//DEBUG.println(buf_p);

uint8_t fbuf[0x1000];
int j;
char mes[0x3];
String msg = "";

  char message[0x80];
  
if ((buf_pf % sizeof buf) != (buf_p % sizeof buf)){
    j = buf_p - buf_pf;
    DEBUG.print(buf_p);
    DEBUG.print(" - ");
    DEBUG.print(buf_pf);
    DEBUG.print(" = ");
    DEBUG.println(j);

      memset(&message, 0, sizeof message);
      sprintf(message, "<BUF:%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x> \r\n", buf[buf_pf], buf[buf_pf+1], buf[buf_pf+2], buf[buf_pf+3], buf[buf_pf+4], buf[buf_pf+5], buf[buf_pf+6], buf[buf_pf+7], buf[buf_pf+8], buf[buf_pf+9]);
      Serial.write(message, 50);
    
    if (j < 0){ j = sizeof buf - buf_pf;} 
    uint8_t b[j];
    memcpy(b,&buf[(buf_pf % sizeof buf)],j);
    file.write(b,sizeof b);
    file.flush();
    for (int i = 0; i < j ; i++){
      sprintf(mes, "%02x " , buf[(buf_pf % sizeof buf)]);
      msg.concat(mes);
      buf_pf++;
      } 
    DEBUG.println(msg);
    }
    
}

void led_fail(int i){
  while(1){
     ledon();
     delay(i);
     ledoff();
     delay(i);
    }
  }
