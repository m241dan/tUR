/****************************************************
 *  Overview:
 * This program is an example of using the SD library 
 * with some sensor data.  This program will write to 
 * a .txt file using the SD 525kb buffer. In addition,
 * this program contains a simple error handling technique
 * which will prevent data loss from power-loss.
 * 
 *  Notes(For the System Lead to Read):
 *  - Uses the standard Arduino SD library 
 *  - 525kb buffer
 *  - SPI
 *  - CSV file data format
 *  - Checks for existing files
 *  -Only need CS pin to control the rest just plug and play
 * 
 */

 #include <SD.h>
 #include <SPI.h>

 //The file setup 
 File myFile;
 //Iterator
 int fileNum = 0;
 //String to storee the file name can be char as well
 String data = "flight.txt";  
 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
 if(!SD.begin(10)){
  Serial.println("initialization failed");
  return;
 }

 
 

 while(SD.exists(data))
  {
    Serial.println("File exists");
    fileNum++;
    data = "flight" + String(fileNum) + ".txt";
    Serial.println("File number is: " + String(fileNum));
    delay(500);
    
    
    

  }
  
   Serial.println("File is good to write to");
    
  }

void loop() {

  float sensors[6] = {5.6, -3.2, 56.3, 9324.12, -34555.4, 23.2};
  writeData(sensors);
}

void writeData(float sensors[]){

   
  myFile = SD.open(data, FILE_WRITE);
    if(myFile){
      for (int i = 0; i < 6; i++){
        myFile.print(sensors[i]);
        myFile.print(",");
        
        delay(500);
        Serial.println(String(i) + " data entered");
        delay(1000);
      }

      myFile.println();
      myFile.close();
      Serial.println("Closed");
      delay(500);
    }
  
  }


