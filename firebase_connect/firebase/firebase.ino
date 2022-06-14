#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <FirebaseArduino.h>
#include "string.h"
#include <SoftwareSerial.h>


#define FIREBASE_HOST "send-receive-gcode-default-rtdb.firebaseio.com" 
#define FIREBASE_AUTH "PJxVu6obGvSuc8ayf3msvUpCsS2MlqQTDD8nUnRQ"
#define WIFI_SSID "123456"
#define WIFI_PASSWORD "12345678"


SoftwareSerial mySerial(15, 13);
char data_received;
String data_buffer;
String file[100];
String fireStatus = "";
char status_array[512];
int fireStatus_length,size_file = 0;
boolean newData = false;

void Readline();

void setup() 
{
  Serial.begin(9600);
  mySerial.begin(9600);
  delay(1000);                
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Serial.print("IP Address is : ");
  Serial.println(WiFi.localIP());
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  delay(1000);
} 

void loop() {
    int number_package = Firebase.getString("number").toInt();
    int index_package = 1;
    fireStatus_length = Firebase.getString(String(1)).length();
    fireStatus = Firebase.getString("1");
    fireStatus.toCharArray(status_array,fireStatus_length+1);
    Readline();
    delay(10000);
    Serial.print("package ");
    Serial.print(index_package);
    Serial.println(":");
    Firebase.setString("type_data", "old");
    Serial.print("data send from esp: ");
    mySerial.print(file[0]);
    mySerial.print("\n");
    Serial.println(file[0]);
//    while(data_received != '\n'){
//      data_received = mySerial.read();
//      data_buffer+= data_received;
//      Serial.println(data_buffer);
//      mySerial.println(file[0]);
//      Serial.println(file[0]);
//      //delay(2000);
//    }
//    Serial.println("escape");
    int a = 1; // index of command in each packet
    //delay(5000);
    //chương trình chỉ thực hiện bên trong vòng lặp này
  while(1){
    String type_data = Firebase.getString("type_data"); // kiểm tra app có gửi dữ liệu mới đến database không
    
    if(type_data == "new"){                                           // nếu có dữ liệu mới thì gửi lại từ package ban đầu
      int number_package = Firebase.getString("number").toInt();      //số lượng package của toàn bộ dữ liệu
      int index_package = 1;                                          // chỉ số của từng package
      fireStatus_length = Firebase.getString(String(1)).length();
      fireStatus = Firebase.getString("1");
      fireStatus.toCharArray(status_array,fireStatus_length+1);
      Readline();
      Serial.print("package ");
      Serial.print(index_package);
      Serial.println(":");
      //Serial.println(status_array);
      Firebase.setString("type_data", "old");         // nhân được dữ liệu từ database thì xem nó là dữ liệu cũ
      Serial.print("data send from esp: ");
      mySerial.print(file[0]);
      mySerial.print("\n");
      Serial.println(file[0]);
      int a = 1; // index of command in each packet
    }
    
    //sending package i + 1 after completing sending package i
    if(a == size_file){
      if(index_package < number_package){

        // read the next package
        index_package++;
        fireStatus_length = Firebase.getString(String(index_package)).length();
        fireStatus = Firebase.getString(String(index_package));
        fireStatus.toCharArray(status_array,fireStatus_length+1);
        Readline();
        Serial.print("package ");
        Serial.print(index_package);
        Serial.println(":");
        Serial.print("data send from esp: ");
        mySerial.print(file[0]);
        mySerial.print("\n");
        Serial.println(file[0]);
        a = 1;
      }
      else if(index_package == number_package){
        mySerial.print("home");
        mySerial.print("\n");
        Serial.println("home");
        while(type_data == "old"){
          type_data = Firebase.getString("type_data");
        }
        index_package++;
      }
    }
     
     //send commands to stm32 in each package
      while(a<size_file){
        while(mySerial.available()>0 && newData == false){
          data_received = mySerial.read();
          data_buffer+= data_received;
          if(data_received == '\n'){
            Serial.print("data received from stm32: ");
            Serial.println(data_buffer);
            if(data_buffer != "ok\n"){
              data_buffer = "";
            }
            else{            //received "ok\n" from stm32
              data_buffer = "";
              newData = true;
              Serial.print("data send from esp: ");     //send the next package command
              Serial.println(file[a]);
              mySerial.print(file[a]);
              mySerial.print("\n");
              a++;
              newData = false;
            }
          }
        }
      }
   }
}


void Readline(){
  int i = 0,x=0,y = 0;
  char file_temp[10][30];
  while(status_array[i] != NULL){
        if(status_array[i] == ';'){
            file_temp[x][y] = status_array[i];
            file_temp[x][y+1] = '\0';
            file[x] =  file_temp[x];
            x++;
            y = 0;
        }
        else{
            file_temp[x][y] = status_array[i];
            y++;
        }
        i++;
    }
  size_file = x;   
}
