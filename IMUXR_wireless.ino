#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
//for Wifi capabilities
#include <Arduino_JSON.h>
#include <HTTPClient.h>
#include <WiFi.h>  // Use WiFi.h for ESP32
#include <time.h>


//#define USE_SPI       // Uncomment this to use SPI
#define SERIAL_PORT Serial
#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined
#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 1 // The value of the last bit of the I2C address, default is 1
#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

//Wifi and CWRUXR information
typedef struct {
  const char* ssid = "CaseRegistered";
  const char* password = "";
  String serverUrl = "https://cwruxrstudents.azurewebsites.net/api/v2";
  String roomName = "Jia";
  String anchorID = "Anchor1";
  String apikey = "FCF0D275B2AA0769DC1D7F7941824E";
  String anchorUrl = serverUrl + "/anchor";
  String objectUrl = serverUrl + "/object";
} cwruxr_config;

// Initialize sensor data variables as globals to reference outside of void loop()
float q0 = 0;
float q1 = 0;
float q2 = 0;
float q3 = 0;
float ax = 0;
float ay = 0;
float az = 0;
float gx = 0;
float gy = 0;
float gz = 0;
float mx = 0;
float my = 0;
float mz = 0;

typedef struct {
  String id = "MyObject";
  double xpos = 0.0;
  double ypos = 1.0;
  double zpos = 0.0;
  double xrot = 0.0;
  double yrot = 0.0;
  double zrot = 0.0;
  double wrot = 1.0;
  double xscale = 0.2;
  double yscale = 0.1;
  double zscale = 0.3;
} cube_struct;

cube_struct DisplayStruct;
cwruxr_config my_cwruxr_config;
HTTPClient http;
unsigned long LastSend = 0;
int ischange=0;

void setup()
{
  Serial.begin(115200); // Start the serial console
  Serial.println(F("ICM-20948"));
  delay(100);
  while (Serial.available()) // Make sure the serial RX buffer is empty
    Serial.read();

//---------------------------------sensor set up-----------------------------------
#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif
  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
  bool initialized = false;
  while (!initialized)
  {
    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println(F("Trying again..."));
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  Serial.println(F("Device connected!"));
  bool success = true; // Use success to show if the DMP configuration was successful
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  // Enable the DMP Game Rotation Vector sensor (Quat6)
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  // Enable additional sensors / features
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 10) == ICM_20948_Stat_Ok);        // Set to 5Hz
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 54) == ICM_20948_Stat_Ok);        // Set to 1Hz
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 54) == ICM_20948_Stat_Ok);         // Set to 1Hz
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 54) == ICM_20948_Stat_Ok);  // Set to 1Hz
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 54) == ICM_20948_Stat_Ok);        // Set to 1Hz
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 54) == ICM_20948_Stat_Ok); // Set to 1Hz
  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
  // Check success
  if (success)
  {
    SERIAL_PORT.println(F("DMP enabled!"));
  }
  else
  {
    Serial.println(F("Enable DMP failed!"));
    Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // Do nothing more
  }
//-----------------------------http and wifi set up-----------------------------------
  http.setReuse(true);
  //Establish Wifi Connection
  WiFi.begin(my_cwruxr_config.ssid, my_cwruxr_config.password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());
  delay(10000); //added delay for the user to read the ip address of the website 
  
  CreateCubeJSON(DisplayStruct);  //start a json message for the CWRUXR cube object
} //end of void setup() ---------------------------------------------------------------

void loop()
{
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);
  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Check for valid data
  {
    if (((data.header & DMP_header_bitmap_Quat6) > 0) 
    && ((data.header & DMP_header_bitmap_Accel) > 0) 
    && ((data.header & DMP_header_bitmap_Gyro) > 0) 
    && ((data.header & DMP_header_bitmap_Compass) > 0))
    {
      //Serial.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);
      q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0 ; // Convert to double. Divide by 2^30
      q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0 ; // Convert to double. Divide by 2^30
      q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0 ; // Convert to double. Divide by 2^30
      q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
      // Extract the raw accelerometer data in (mg) and convert it to m/s^2
      ax = (float)data.Raw_Accel.Data.X/1000*9.80665; 
      ay = (float)data.Raw_Accel.Data.Y/1000*9.8066;
      az = (float)data.Raw_Accel.Data.Z/1000*9.8066;
      // Extract the raw gyro data (DPS) and convert units to Radians
      gx = (float)data.Raw_Gyro.Data.X/0.017453; 
      gy = (float)data.Raw_Gyro.Data.Y/0.017453;
      gz = (float)data.Raw_Gyro.Data.Z/0.017453;
      mx = (float)data.Compass.Data.X; // Extract the compass data
      my = (float)data.Compass.Data.Y;
      mz = (float)data.Compass.Data.Z;
      //Print the data
      Serial.print(F("Rotation Quaternion[ "));
      Serial.print(q1, 3);
      Serial.print(F(","));
      Serial.print(q2, 3);
      Serial.print(F(","));
      Serial.print(q3, 3);
      Serial.print(F(","));
      Serial.print(q0, 3);
      Serial.println(F(" ] "));
      Serial.print(F("Acc (m/s^2) [ "));
      Serial.print(ax);
      Serial.print(F(","));
      Serial.print(ay);
      Serial.print(F(","));
      Serial.print(az);
      Serial.println(F(" ] "));
      Serial.print(F("Gyr (Radians) [ "));
      Serial.print(gx);
      Serial.print(F(","));
      Serial.print(gy);
      Serial.print(F(","));
      Serial.print(gz);
      Serial.println(F(" ] "));
      Serial.print(F("Compass (uT) [ "));
      Serial.print(mx);
      Serial.print(F(","));
      Serial.print(my);
      Serial.print(F(","));
      Serial.print(mz);
      Serial.println(F(" ] "));
  
      DisplayStruct.xrot=q1;
      DisplayStruct.yrot=q3;
      DisplayStruct.zrot=q2;
      DisplayStruct.wrot=-(q0);

      ischange=1;
    }
  }//end of checking for valid data 

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away and not delay
  {
    delay(10);
  }
  
  // Put in a slight delay to help debounce the reading
  if ((millis()>(LastSend+100)) && ischange==1)
  {
    SendObject(my_cwruxr_config, CreateCubeJSON(DisplayStruct));
    LastSend=millis();
    ischange=0;
  }
  delay (2);
}

//** Create the JSON for the ramp
JSONVar CreateCubeJSON(cube_struct spst) 
{
  JSONVar position_value;
    position_value["x"] = spst.xpos;
    position_value["y"] = spst.ypos;
    position_value["z"] = spst.zpos;
  JSONVar rotation_value;
    rotation_value["x"] = spst.xrot;
    rotation_value["y"] = spst.yrot;
    rotation_value["z"] = spst.zrot;
    rotation_value["w"] = spst.wrot;
  JSONVar scale_value;
    scale_value["x"] = spst.xscale;
    scale_value["y"] = spst.yscale;
    scale_value["z"] = spst.zscale;
  JSONVar positionObj = position_value;
  JSONVar rotationObj = rotation_value;
  JSONVar scaleObj = scale_value;
  JSONVar poseObj;
    poseObj["position"] = positionObj;
    poseObj["rotation"] = rotationObj;
    poseObj["scale"] = scaleObj;
  JSONVar Obj;
    Obj["id"] = spst.id;
    Obj["anchorID"] = "";
    Obj["parentID"] = "";
    Obj["type"] = "Primitive";
    Obj["source"] = "Cube";
    Obj["active"] = true;
    Obj["materialID"] = "Lit:White";
    Obj["pose"] = poseObj;
    Obj["isManipulationOn"]=false;
    Obj["useBoundsControl"]=true;
    Obj["walkable"]=false;
    Obj["isColliderOn"]=true;
  // #ifdef DEBUG
  //     Serial.println(JSON.stringify(Obj));
  // #endif
    return Obj;
}

//** Send an object structure via the crwuxr api using a stringified JSON
int SendObject(cwruxr_config cnfig, JSONVar JSONString) 
{
     //Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED)
    {
      int httpResponse;
      Serial.print("Object URL: ");
      Serial.println(cnfig.objectUrl);
      //Serial.print("Room ID: ");
      //Serial.println(cnfig.roomName);
      //Serial.print("Anchor ID: ");
      //Serial.println(cnfig.anchorID);
      //Serial.print("API Key: ");
      //Serial.println(cnfig.apikey);
      
      //http end points and header
      http.begin(cnfig.objectUrl);
      http.addHeader("accept", "*/*");
      http.addHeader("ApiKey", cnfig.apikey);
      http.addHeader("RoomId", cnfig.roomName);
      http.addHeader("AnchorId", cnfig.anchorID);
      http.addHeader("Content-Type", "application/json-patch+json");
      httpResponse = http.POST(JSON.stringify(JSONString));
      
      #ifdef DEBUG
      if (httpResponse > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponse);
        String response = http.getString();
        Serial.println(response);
      } 
      else 
      {
        Serial.print("Error code: ");
        Serial.println(httpResponse);
      }
      #endif

      http.end();

      if (httpResponse == 200) 
      {
        return true;
      } 
      else 
      {
        return false;
      }
    } 
    else 
    {
      Serial.println("WiFi disconnected!");
    }
  
}
