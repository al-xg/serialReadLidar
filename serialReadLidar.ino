/*
http://xv11hacking.wikispaces.com/LIDAR+Sensor
 
 A full revolution will yield 90 packets, containing 4 consecutive readings each.
 The length of a packet is 22 bytes.
 This amounts to a total of 360 readings (1 per degree) on 1980 bytes.
 
 Each packet is organized as follows:
 <start> <index> <speed_L> <speed_H> [Data 0] [Data 1] [Data 2] [Data 3] <checksum_L> <checksum_H>
 */



const int NumSectors=6;
byte Sector=0;
byte packetCount=0;

byte minDistIndex = 0;
unsigned long SectorMinDist =0;
unsigned long minDist=0;
byte i=0;


byte b=0;
byte init_level=0;
byte packet_index;
unsigned long data[21];
byte data_index=0;

//unsigned long checksum[2];

unsigned long  SpeedRPH;
//unsigned long  Distance[4], Quality[4];
unsigned long   SectorData[15], LidarData[6];

unsigned long report_time;
unsigned long tmp_time;



void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop(){
  if (Serial1.available() > 7) {
    decode_data();
  }
  tmp_time=millis();
  if (tmp_time > report_time+400) {
    //Serial.print(".");
    report_time=millis();
  }
}



void decode_data(){
  if (init_level == 0) {
    //header
    b = Serial1.read();

    if (b == 0xFA ){
      //Serial.println("level1");
      init_level = 1;
    }
    else init_level = 0;
  }

  else if (init_level == 1){
    // position index                  
    b = Serial1.read();
    if (b >= 0xA0 & b <= 0xF9){ //is the index byte in the 90 packets, going from 0xA0 (packet 0, readings 0 to 3) to 0xF9 (packet 89, readings 356 to 359).
      packet_index = b - 0xA0;
      //Serial.println(packet_index);
      init_level = 2;
      //Serial.println("level2");

      //Sync sector count start with 0degrees
      if (packet_index==0) {
        Sector=0;
        packetCount=0;
      }

      
      if (packetCount>=(14)){ //Then the last packet of the sector has been reached...
        packetCount=0;

        //Work out smallest distance reading of this Sector
        minDistIndex = 0;
        SectorMinDist= SectorData[minDistIndex];
        for (i=0; i<15; i++){
          if (SectorMinDist<SectorData[i]){
            SectorMinDist = SectorData[i];
            minDistIndex = i;
          }
        }
        
        Serial.print("Sector: ");
        Serial.print(Sector);
        Serial.print("  MinDist: ");
        Serial.println(SectorMinDist);
        
        LidarData[Sector]=SectorMinDist;
        //LidarData[6]='\0';
        
        //Move to the next Sector
        Sector++;
      }
      packetCount++;
    }
    else if (b != 0xFA){
      init_level = 0;
    }
  }

  else if (init_level == 2){
    //speed (2 bytes), 4x distance(4 bytes each), checksum (2 bytes)
    for (data_index=0; data_index <20; data_index++){ //store 20 bytes
      data[data_index]=Serial1.read();
      //Serial.println(data_index);
    }
    //data[data_index]='\0';

    //two-byte information, little-endian. It represents the speed, in 64th of RPM (aka value in RPM represented in fixed point, with 6 bits used for the decimal part).
    SpeedRPH=(data[0]<<8)|data[1];
    
    //For each consecutive reading:
      //`byte 0 : <distance 7:0>` 14bit
      //`byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8>`
      //`byte 2 : <signal strength 7:0>`
      //`byte 3 : <signal strength 15:8>`
    
    //Reading 1
    //Distance[0]=(data[2] | (data[3] & 0x3f)<<8);
    //Quality[3]= data[4] | (data[5] << 8);
    
    minDist=(data[2] | (data[3] & 0x3f)<<8);
    
    //Reading 2
    //Distance[1]=(data[6] | (data[7]& 0x3f)<<8);
    //Quality[3]= data[8] | (data[9] << 8);
    
    if ((data[6] | (data[7]& 0x3f)<<8)<minDist) {minDist=(data[6] | (data[7]& 0x3f)<<8);}
    
    //Reading 3
    //Distance[2]=(data[10] | (data[11]& 0x3f)<<8);
    //Quality[3]= data[12] | (data[13] << 8);
    
    if ((data[10] | (data[11]& 0x3f)<<8)<minDist) {minDist=(data[10] | (data[11]& 0x3f)<<8);}
    
    //Reading 4
    //Distance[3]=(data[14] | (data[15]& 0x3f)<<8);
    //Quality[3]= data[16] | (data[17] << 8);
    
    
    if ((data[14] | (data[15]& 0x3f)<<8)<minDist) {minDist=(data[14] | (data[15]& 0x3f)<<8);}

  //Distance[4]='\0';
    //Quality[4]='\0';
    
    /* //Work out the minimum distance of this packet
    minDistIndex = 0;
    minDist= Distance[minDistIndex];
    for (i=0; i<4; i++){
      if (minDist<Distance[i]){
        minDist = Distance[i];
        minDistIndex = i;
      }
    }
    */
    SectorData[packetCount]=minDist;
    //SectorData[15]='\0';

    //Checksum for this packet
    //checksum[0]=data[18];
    //checksum[1]=data[19];
    
    //Move to the next packet
    init_level=0;
  }  

  else init_level = 0; // default, should never happen...

}


