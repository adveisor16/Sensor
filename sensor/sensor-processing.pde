import processing.serial.*;

int lf = 10; // Ascii for line feed
Serial myPort;
String myString;

void setup() {
  myPort = new Serial(this, Serial.list()[0], 9600);
  myPort.clear();
  size(800,600);
  background(255);
}

void draw() {
  if ( myPort.available() > 0 ) {
    myString = myPort.readStringUntil(lf);
    if ( myString != null ) {
      //println(split(trim(myString), ",")[0]);      
      String[] sAccel = split(myString, ",");
      if ( sAccel.length == 3 ) {
        float x = Float.parseFloat(sAccel[0])/100;
        float y = Float.parseFloat(sAccel[1])/100;
        float z = Float.parseFloat(sAccel[2])/100;
        println("x: " + x + ", y: " + y + ", z: " + z);
        
        clearScreen();
        float startH = 300; // Ursprung (0,0), horizontal
        
        textSize(60);
        
        // Draw x chart
        fill(245,1,1);
        stroke(245,1,1);
        text("X", 10, 60);
        rect(startH + x, 20, abs(x), 80);
        
        // Draw y chart
        fill(125,252,5);
        stroke(125,252,5);
        text("Y", 20, 200);
        rect(startH + y, 160, abs(y), 80);
        
        // Draw z chart
        fill(115,45,235);
        stroke(115,45,235);
        text("Z", 20, 340);
        rect(startH + x, 300, abs(z), 80);
      }
    }
  }
}

void clearScreen() {
  background(255); 
}