/*
http://xv11hacking.wikispaces.com/LIDAR+Sensor

A full revolution will yield 90 packets, containing 4 consecutive readings each.
The length of a packet is 22 bytes.
This amounts to a total of 360 readings (1 per degree) on 1980 bytes.

Each packet is organized as follows:
<start> <index> <speed_L> <speed_H> [Data 0] [Data 1] [Data 2] [Data 3] <checksum_L> <checksum_H>
*/




unsigned char b=0;
byte init_level=0;
byte packet_index;
unsigned char data[21];
byte data_index=0;

unsigned char b_speed[2], b_data0[4], b_data1[4], b_data2[4], b_data3[4], checksum[2];

double SpeedRPH;
double  Distance[4], Quality[4];

unsigned long report_time;
unsigned long tmp_time;



void setup() {
  Serial.begin(115200);
}

void loop(){
  if (Serial.available() > 0) {
    decode_data();
    }
    tmp_time=millis();
    if (tmp_time > report_time+400) {
      Serial.print(".");
      report_time=millis();
  }
}



void decode_data(){
  if (init_level == 0) {
    //header
    b = Serial.read();

    if (b == 0xFA ){
      //Serial.println("level1");
      init_level = 1;
    }
    else init_level = 0;
  }

  else if (init_level == 1){
    // position index
    b = Serial.read();
    if (b >= 0xA0 & b <= 0xF9){ //is the index byte in the 90 packets, going from 0xA0 (packet 0, readings 0 to 3) to 0xF9 (packet 89, readings 356 to 359).
      packet_index = b - 0xA0;
      init_level = 2;
      //Serial.println(packet_index);
      //Serial.println("level2");
    }
    else if (b != 0xFA){
      init_level = 0;
    }
  }

 else if (init_level == 2){
    //speed (2 bytes), 4x distance(4 bytes each), checksum (2 bytes)
    for (data_index=0; data_index <20; data_index++){ //store 20 bytes
      data[data_index]=Serial.read();
      //Serial.println(data_index);
    }
    data[data_index]='\0';
    
    //b_speed[0]=data[0];  //two-byte information, little-endian. It represents the speed, in 64th of RPM (aka value in RPM represented in fixed point, with 6 bits used for the decimal part).
    //b_speed[1]=data[1];
    SpeedRPH=(data[0]<<8)|data[1];
    
    //b_data0[0]=data[2];  //`byte 0 : <distance 7:0>` 14bit
    //b_data0[1]=data[3];  //`byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8>`
    Distance[0]=(data[2] | (data[3]& 0x3f)<<8);
    //b_data0[2]=data[4];  //`byte 2 : <signal strength 7:0>`
    //b_data0[3]=data[5];  //`byte 3 : <signal strength 15:8>`
    Quality[3]= data[4] | (data[5] << 8);
    
    //b_data1[0]=data[6];
    //b_data1[1]=data[7];
    Distance[1]=(data[6] | (data[7]& 0x3f)<<8);
    //b_data1[2]=data[8];
    //b_data1[3]=data[9];
    Quality[3]= data[8] | (data[9] << 8);
    
    //b_data2[0]=data[10];
    //b_data2[1]=data[11];
    Distance[2]=(data[10] | (data[11]& 0x3f)<<8);
    //b_data2[2]=data[12];
    //b_data2[3]=data[13];
    Quality[3]= data[12] | (data[13] << 8);
    
    //b_data3[0]=data[14];
    //b_data3[1]=data[15];
    Distance[3]=(data[14] | (data[15]& 0x3f)<<8);
    //b_data3[2]=data[16];
    //b_data3[3]=data[17];
    Quality[3]= data[16] | (data[17] << 8);
    
    checksum[0]=data[18];
    checksum[1]=data[19];
    
    init_level=0;
  }  

  else init_level = 0; // default, should never happen...
}




