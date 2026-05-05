#include <Arduino.h>
#include <WiFiNINA.h>

#define MOTOR1_EN 4
#define MOTOR1_IN1 2
#define MOTOR1_IN2 3

#define MOTOR2_EN 7
#define MOTOR2_IN1 5
#define MOTOR2_IN2 6

#define MOTOR3_EN 8
#define MOTOR3_IN1 9
#define MOTOR3_IN2 10

#define DEBUG 1

// WiFi configuration
char ssid[] = "WXP_Robot";  // SSID for the Arduino's WiFi AP
char pass[] = "12345678";   // Password for the Arduino's WiFi AP
IPAddress local_ip(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiServer server(80);
int status = WL_IDLE_STATUS;

struct motion_vector {
  float angle_degrees;
  float speed; // Speed (m/s) - adjust as needed for your robot's capabilities
} motion_vector;

float mass = 2.0; // Mass of the robot (kg) - adjust as needed for more accurate physics calculations

struct Impulse {
  float time;
  float angle;
  float magnitude;
};

const int MAX_IMPULSES = 20;
Impulse impulseList[MAX_IMPULSES];
int impulseCount = 0;
bool impulseRunning = false;
unsigned long runStartMillis = 0;
int nextImpulseIndex = 0;
String impulseStatus = "stopped";

void set_motion(int bearing_degrees, float speed) {

  // Angle offsets for wheels on each corner of the robot
  float theta_A = bearing_degrees + 180.0;
  float theta_B = bearing_degrees + 60.0;
  float theta_C = bearing_degrees - 60.0;

  // Calculate motor speeds based on the desired bearing and speed
  float motor1_speed = (speed * (float)255) * sin(theta_A * (PI / 180.0));
  float motor2_speed = (speed * (float)255) * sin(theta_B * (PI / 180.0));
  float motor3_speed = (speed * (float)255) * sin(theta_C * (PI / 180.0));

  // Convert motor speeds to integers for PWM output
  int motor1_speed_int = (int) round(motor1_speed);
  int motor2_speed_int = (int) round(motor2_speed);
  int motor3_speed_int = (int) round(motor3_speed);

  // Debugging output to Serial Monitor
  #if DEBUG
  Serial.println("Calculated motor speeds:");
  Serial.print("Motor 1: ");
  Serial.println(motor1_speed);
  Serial.print("Motor 1 as int: ");
  Serial.println(motor1_speed_int);

  Serial.print("Motor 2: ");
  Serial.println(motor2_speed);
  Serial.print("Motor 2 as int: ");
  Serial.println(motor2_speed_int);

  Serial.print("Motor 3: ");
  Serial.println(motor3_speed);
  Serial.print("Motor 3 as int: ");
  Serial.println(motor3_speed_int);
  #endif

  // Set motor directions and speeds based on the calculated values
  if (motor1_speed_int > 0) {
    #if DEBUG
    Serial.println("Motor 1 forward");
    #endif
    digitalWrite(MOTOR1_EN, HIGH);
    analogWrite(MOTOR1_IN1, motor1_speed_int);
    analogWrite(MOTOR1_IN2, 0);
  } else if (motor1_speed_int < 0) {
    #if DEBUG
    Serial.println("Motor 1 backward");
    #endif
    digitalWrite(MOTOR1_EN, HIGH);
    analogWrite(MOTOR1_IN1, 0);
    analogWrite(MOTOR1_IN2, abs(motor1_speed_int));
  } else {
    #if DEBUG
    Serial.println("Motor 1 stopped");
    #endif
    digitalWrite(MOTOR1_EN, LOW);
    analogWrite(MOTOR1_IN1, 0);
    analogWrite(MOTOR1_IN2, 0);
  }

  if (motor2_speed_int > 0) {
    #if DEBUG
    Serial.println("Motor 2 forward");
    #endif
    digitalWrite(MOTOR2_EN, HIGH);
    analogWrite(MOTOR2_IN1, motor2_speed_int);
    analogWrite(MOTOR2_IN2, 0);
  } else if (motor2_speed_int < 0) {
    #if DEBUG
    Serial.println("Motor 2 backward");
    #endif
    digitalWrite(MOTOR2_EN, HIGH);
    analogWrite(MOTOR2_IN1, 0);
    analogWrite(MOTOR2_IN2, abs(motor2_speed_int));
  } else {
    #if DEBUG
    Serial.println("Motor 2 stopped");
    #endif
    digitalWrite(MOTOR2_EN, LOW);
    analogWrite(MOTOR2_IN1, 0);
    analogWrite(MOTOR2_IN2, 0);
  }

  if (motor3_speed_int > 0) {
    #if DEBUG
    Serial.println("Motor 3 forward");
    #endif
    digitalWrite(MOTOR3_EN, HIGH);
    analogWrite(MOTOR3_IN1, motor3_speed_int);
    analogWrite(MOTOR3_IN2, 0);
  } else if (motor3_speed_int < 0) {
    #if DEBUG
    Serial.println("Motor 3 backward");
    #endif
    digitalWrite(MOTOR3_EN, HIGH);
    analogWrite(MOTOR3_IN1, 0);
    analogWrite(MOTOR3_IN2, abs(motor3_speed_int));
  } else {
    #if DEBUG
    Serial.println("Motor 3 stopped");
    #endif
    digitalWrite(MOTOR3_EN, LOW);
    analogWrite(MOTOR3_IN1, 0);
    analogWrite(MOTOR3_IN2, 0);
  }
}

void apply_impulse(float impulse_magnitude, float impulse_angle_degrees) {
  // Calculate the angle of the desired motion vector

  #if DEBUG
  Serial.println("Applying impulse:");
  Serial.print("Impulse magnitude: ");
  Serial.println(impulse_magnitude);
  Serial.print("Impulse angle: ");
  Serial.println(impulse_angle_degrees);
  #endif

  float speed_x = motion_vector.speed * sin(motion_vector.angle_degrees * (PI / 180.0));
  float speed_y = motion_vector.speed * cos(motion_vector.angle_degrees * (PI / 180.0));

  #if DEBUG
  Serial.println("Existing motion scalars:");
  Serial.print("X: ");
  Serial.println(speed_x);
  Serial.print("Y: ");
  Serial.println(speed_y);
  #endif

  float impulse_x = impulse_magnitude * sin(impulse_angle_degrees * (PI / 180.0));
  float impulse_y = impulse_magnitude * cos(impulse_angle_degrees * (PI / 180.0));

  #if DEBUG
  Serial.println("Applied impulse scalars:");
  Serial.print("X: ");
  Serial.println(impulse_x);
  Serial.print("Y: ");
  Serial.println(impulse_y);
  #endif

  float new_speed_x = speed_x + (impulse_x / mass);
  float new_speed_y = speed_y + (impulse_y / mass);

  #if DEBUG
  Serial.println("New motion scalars:");
  Serial.print("X: ");
  Serial.println(new_speed_x);
  Serial.print("Y: ");
  Serial.println(new_speed_y);
  #endif

  float new_speed = sqrt(new_speed_x * new_speed_x + new_speed_y * new_speed_y);
  float new_angle_degrees = atan2(new_speed_x, new_speed_y) * (180.0 / PI);

  #if DEBUG
  Serial.println("New motion vector:");
  Serial.print("Speed: ");
  Serial.println(new_speed);
  Serial.print("Angle: ");
  Serial.println(new_angle_degrees);
  #endif

  motion_vector.angle_degrees = new_angle_degrees;
  motion_vector.speed = new_speed;

  #if DEBUG
  Serial.println("Setting new motion vector:");
  Serial.print("Speed: ");
  Serial.println(motion_vector.speed);
  Serial.print("Angle: ");
  Serial.println(motion_vector.angle_degrees);
  #endif

  set_motion(new_angle_degrees, new_speed);
}

String urlDecode(const String &src) {
  String decoded = "";
  for (int i = 0; i < src.length(); i++) {
    char c = src[i];
    if (c == '+') {
      decoded += ' ';
    } else if (c == '%' && i + 2 < src.length()) {
      char high = src[i + 1];
      char low = src[i + 2];
      int value = 0;
      if (high >= '0' && high <= '9') value += (high - '0') * 16;
      else if (high >= 'A' && high <= 'F') value += (high - 'A' + 10) * 16;
      else if (high >= 'a' && high <= 'f') value += (high - 'a' + 10) * 16;
      if (low >= '0' && low <= '9') value += (low - '0');
      else if (low >= 'A' && low <= 'F') value += (low - 'A' + 10);
      else if (low >= 'a' && low <= 'f') value += (low - 'a' + 10);
      decoded += char(value);
      i += 2;
    } else {
      decoded += c;
    }
  }
  return decoded;
}

String trimString(const String &value) {
  int start = 0;
  int end = value.length() - 1;
  while (start <= end && isspace(value[start])) start++;
  while (end >= start && isspace(value[end])) end--;
  if (start > end) return "";
  return value.substring(start, end + 1);
}

bool parseImpulseLine(const String &line, Impulse &imp) {
  String trimmed = trimString(line);
  if (trimmed.length() == 0) return false;
  if (trimmed.startsWith("#")) return false;

  String normalized = trimmed;
  for (int i = 0; i < normalized.length(); i++) {
    if (normalized[i] == ',') normalized[i] = ' ';
  }

  String parts[3];
  int partIndex = 0;
  String token = "";
  for (int i = 0; i < normalized.length() && partIndex < 3; i++) {
    char c = normalized[i];
    if (isspace(c)) {
      if (token.length() > 0) {
        parts[partIndex++] = trimString(token);
        token = "";
      }
    } else {
      token += c;
    }
  }
  if (token.length() > 0 && partIndex < 3) {
    parts[partIndex++] = trimString(token);
  }

  if (partIndex < 3) return false;

  imp.time = parts[0].toFloat();
  imp.angle = parts[1].toFloat();
  imp.magnitude = parts[2].toFloat();
  return true;
}

int parseImpulseList(const String &body) {
  String data = body;
  int count = 0;
  String line = "";

  for (int i = 0; i < data.length(); i++) {
    char c = data[i];
    if (c == '\r') continue;
    if (c == '\n') {
      if (line.length() > 0) {
        Impulse imp;
        if (parseImpulseLine(line, imp) && count < MAX_IMPULSES) {
          impulseList[count++] = imp;
        }
      }
      line = "";
    } else {
      line += c;
    }
  }
  if (line.length() > 0) {
    Impulse imp;
    if (parseImpulseLine(line, imp) && count < MAX_IMPULSES) {
      impulseList[count++] = imp;
    }
  }
  return count;
}

void stopImpulseRun() {
  impulseRunning = false;
  impulseCount = 0;
  nextImpulseIndex = 0;
  impulseStatus = "stopped";
  set_motion(0, 0);
}

void processImpulses() {
  if (!impulseRunning || impulseCount == 0) return;

  unsigned long now = millis();
  float elapsed = (now - runStartMillis) / 1000.0;

  while (nextImpulseIndex < impulseCount && elapsed >= impulseList[nextImpulseIndex].time) {
    
    #if DEBUG
    Serial.println("Applying impulse:");
    Serial.print("Impulse magnitude: ");
    Serial.println(impulseList[nextImpulseIndex].magnitude);
    Serial.print("Impulse angle: ");
    Serial.println(impulseList[nextImpulseIndex].angle);
    #endif

    apply_impulse(impulseList[nextImpulseIndex].magnitude, impulseList[nextImpulseIndex].angle);
    nextImpulseIndex++;
    if (nextImpulseIndex >= impulseCount) {
      impulseRunning = false;
      impulseStatus = "completed";
      break;
    }
  }
}

// Web server handler
void handleClient(WiFiClient client) {
  String request = "";
  String path = "/";
  String method = "GET";
  bool isPost = false;
  String postData = "";
  
  #if DEBUG
  Serial.println("Client connected");
  #endif
  
  // Read the request line with timeout
  unsigned long startTime = millis();
  while (client.connected() && (millis() - startTime < 1000)) {
    if (client.available()) {
      char c = client.read();
      request += c;
      if (request.length() > 2 && request.endsWith("\r\n")) {
        break;
      }
    }
  }
  
  #if DEBUG
  Serial.print("Request: ");
  Serial.println(request);
  #endif
  
  // Parse method and path
  int firstSpace = request.indexOf(' ');
  int secondSpace = request.indexOf(' ', firstSpace + 1);
  if (firstSpace > 0 && secondSpace > firstSpace) {
    method = request.substring(0, firstSpace);
    path = request.substring(firstSpace + 1, secondSpace);
    isPost = (method == "POST");
  }
  
  #if DEBUG
  Serial.print("Method: ");
  Serial.print(method);
  Serial.print(" Path: ");
  Serial.println(path);
  #endif
  
  // Read headers and body
  int contentLength = 0;
  startTime = millis();
  while (client.connected() && (millis() - startTime < 1000)) {
    String line = "";
    while (client.available()) {
      char c = client.read();
      if (c == '\n') break;
      if (c != '\r') line += c;
    }
    
    if (line.length() == 0) break; // Empty line = end of headers
    
    if (line.startsWith("Content-Length:")) {
      contentLength = line.substring(15).toInt();
      #if DEBUG
      Serial.print("Content-Length: ");
      Serial.println(contentLength);
      #endif
    }
  }
  
  // Read POST data if present
  if (isPost && contentLength > 0) {
    int remaining = contentLength;
    startTime = millis();
    while (remaining > 0 && (millis() - startTime < 1000)) {
      if (client.available()) {
        postData += (char)client.read();
        remaining--;
      }
    }
    #if DEBUG
    Serial.print("POST data: ");
    Serial.println(postData);
    #endif
  }
  
  // Route handling
  if (path == "/" || path == "/index.html") {
    String massStr = String(mass, 1);
    String massDisplay = String(mass, 2);
    String statusDisplay = impulseRunning ? "running" : impulseStatus;
    String impulseListSummary = String(impulseCount) + " impulse" + (impulseCount == 1 ? "" : "s");

    String html = "<!DOCTYPE html><html><head><title>WXP Robot Configuration</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:Arial,sans-serif;margin:20px;background-color:#f0f0f0;}";
    html += ".container{background-color:white;padding:20px;border-radius:8px;max-width:600px;margin:0 auto;box-shadow:0 2px 4px rgba(0,0,0,0.1);}";
    html += "h1{color:#333;text-align:center;}.form-group{margin:15px 0;}label{display:block;margin-bottom:5px;color:#555;font-weight:bold;}";
    html += "input[type='number'],textarea{width:100%;padding:8px;border:1px solid #ddd;border-radius:4px;box-sizing:border-box;font-size:16px;}";
    html += "textarea{height:180px;resize:vertical;}button{width:100%;padding:10px;background-color:#4CAF50;color:white;border:none;border-radius:4px;cursor:pointer;font-size:16px;margin-top:10px;}";
    html += "button:hover{background-color:#45a049;}.info{background-color:#e3f2fd;padding:10px;border-radius:4px;margin-top:20px;color:#1976d2;}";
    html += "</style></head><body><div class='container'><h1>WXP Robot Configuration</h1>";

    html += "<div class='info'><p><strong>Current Mass:</strong> " + massDisplay + " kg</p>";
    html += "<p><strong>Impulse status:</strong> " + statusDisplay + "</p>";
    html += "<p><strong>Loaded impulses:</strong> " + impulseListSummary + "</p>";
    html += "</div>";

    html += "<form method='POST' action='/update'><div class='form-group'>";
    html += "<label for='mass'>Robot Mass (kg):</label>";
    html += "<input type='number' id='mass' name='mass' step='0.1' min='0.1' value='" + massStr + "' required>";
    html += "</div><button type='submit'>Update Mass</button></form>";

    html += "<form method='POST' action='/run'><div class='form-group'>";
    html += "<label for='impulses'>Impulses (#time, angle, magnitude):</label>";
    html += "<textarea id='impulses' name='impulses' placeholder='#time, angle, magnitude\n1.0, 90, 1\n2.5, 180, 0.5'></textarea>";
    html += "</div><button type='submit'>Run</button></form>";

    html += "<form method='POST' action='/stop'><button type='submit'>Stop</button></form>";

    html += "</div></body></html>";

    client.print("HTTP/1.1 200 OK\r\n");
    client.print("Content-Type: text/html\r\n");
    client.print("Content-Length: ");
    client.print(html.length());
    client.print("\r\n");
    client.print("Connection: close\r\n");
    client.print("\r\n");
    client.print(html);

    #if DEBUG
    Serial.println("Sent HTML response");
    #endif
  } 
  else if (path == "/update" && isPost) {
    // Parse mass from POST data (format: mass=value)
    int massStart = postData.indexOf("mass=");
    if (massStart != -1) {
      String massStr = postData.substring(massStart + 5);
      float new_mass = massStr.toFloat();
      if (new_mass > 0.0) {
        mass = new_mass;
        #if DEBUG
        Serial.print("Mass updated to: ");
        Serial.println(mass);
        #endif
      }
    }
    
    // Send redirect response
    client.print("HTTP/1.1 303 See Other\r\n");
    client.print("Location: /\r\n");
    client.print("Connection: close\r\n");
    client.print("\r\n");
    
    #if DEBUG
    Serial.println("Sent redirect response");
    #endif
  } 
  else if (path == "/run" && isPost) {
    int start = postData.indexOf("impulses=");
    if (start != -1) {
      String payload = postData.substring(start + 9);
      int amp = payload.indexOf('&');
      if (amp != -1) payload = payload.substring(0, amp);
      payload = urlDecode(payload);
      impulseCount = parseImpulseList(payload);
      if (impulseCount > 0) {
        impulseRunning = true;
        nextImpulseIndex = 0;
        runStartMillis = millis();
        impulseStatus = "running";
        #if DEBUG
        Serial.print("Loaded ");
        Serial.print(impulseCount);
        Serial.println(" impulses");
        #endif
      } else {
        impulseRunning = false;
        impulseStatus = "no valid impulses";
        #if DEBUG
        Serial.println("No valid impulses loaded");
        #endif
      }
    }
    client.print("HTTP/1.1 303 See Other\r\n");
    client.print("Location: /\r\n");
    client.print("Connection: close\r\n");
    client.print("\r\n");
  } 
  else if (path == "/stop" && isPost) {
    stopImpulseRun();
    client.print("HTTP/1.1 303 See Other\r\n");
    client.print("Location: /\r\n");
    client.print("Connection: close\r\n");
    client.print("\r\n");
  } 
  else {
    String notFound = "404: Not Found";
    client.print("HTTP/1.1 404 Not Found\r\n");
    client.print("Content-Type: text/plain\r\n");
    client.print("Content-Length: ");
    client.print(notFound.length());
    client.print("\r\n");
    client.print("Connection: close\r\n");
    client.print("\r\n");
    client.print(notFound);
    
    #if DEBUG
    Serial.println("Sent 404 response");
    #endif
  }
  
  delay(10);
  client.stop();
  
  #if DEBUG
  Serial.println("Client disconnected");
  #endif
}

void setup() {

  // Initialize Serial communication for debugging
  delay(1000);
  Serial.begin(9600);
  delay(1000);

  #if DEBUG
  Serial.println("Debug mode enabled");
  Serial.println("Starting...");
  #endif

  // Set motor control pins as outputs and initialize them to LOW
  pinMode(MOTOR1_EN, OUTPUT);
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);

  pinMode(MOTOR2_EN, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);

  pinMode(MOTOR3_EN, OUTPUT);
  pinMode(MOTOR3_IN1, OUTPUT);
  pinMode(MOTOR3_IN2, OUTPUT);

  #if DEBUG
  Serial.println("Initializing motor control pins...");
  #endif

  digitalWrite(MOTOR1_EN, LOW);
  digitalWrite(MOTOR2_EN, LOW);
  digitalWrite(MOTOR3_EN, LOW);

  motion_vector.angle_degrees = 0.0;
  motion_vector.speed = 0.0;

  #if DEBUG
  Serial.println("Initialised motion vector:");
  Serial.print("Speed: ");
  Serial.println(motion_vector.speed);
  Serial.print("Angle: ");
  Serial.println(motion_vector.angle_degrees);
  #endif

  // Initialize WiFi as an Access Point
  #if DEBUG
  Serial.println("Starting WiFi AP...");
  #endif
  
  status = WiFi.beginAP(ssid, pass);
  delay(1000);
  WiFi.config(local_ip, gateway, subnet);

  if (status != WL_AP_LISTENING)
  {
    #if DEBUG
    Serial.println("Creating access point failed");
    #endif
  }
  
  #if DEBUG
  Serial.print("WiFi AP started. SSID: ");
  Serial.println(ssid);
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  #endif

  // Start the server
  server.begin();
  
  #if DEBUG
  Serial.println("Web server started on http://192.168.1.1");
  #endif

  // Serial.println("----------");
  // apply_impulse(1.0, 45.0); // Example impulse of magnitude 1.0 at a 45 degree angle
  // delay(1000);
  // Serial.println("----------");
  // apply_impulse(-1.0, 45.0); // Example impulse of magnitude 1.0 at a 135 degree angle
  // delay(1000);
  // Serial.println("----------");
  // apply_impulse(1.0, 180.0); // Example impulse of magnitude 1.0 at a 180 degree angle
  // delay(1000);
  // Serial.println("----------");
  // apply_impulse(1.0, 0.0); // Example impulse of magnitude 1.0 at a 0 degree angle
  // delay(1000);

}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    handleClient(client);
  }

  processImpulses();

  // compare the previous status to the current status
  if (status != WiFi.status())
  {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED)
    {
      // a device has connected to the AP
      #if DEBUG
      Serial.println("Device connected to AP");
      #endif
    }
    else
    {
      // a device has disconnected from the AP, and we are back in listening mode
      #if DEBUG
      Serial.println("Device disconnected from AP");
      #endif
    }
  }

}