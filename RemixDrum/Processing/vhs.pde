// GNU General Public License 3.0


// Import modules that allow to receive OSC messages 
import oscP5.*;
import netP5.*;
OscP5 osc;
// Setting variables that will receive OSC messages 
int drawx, drawy, drawz;
// Setting the X and Y axis variables 
float x, y;
// X-axis movement speed 
int xStep = 50;
// Y-axis movement speed 
int yStep = 80;
float a, a_;
int num;

// Setting the setup function, used to set the initial properties of the environment 
void setup(){
  
  // OSC server
  osc = new OscP5(this, 7777);
  
  // Canvas size
  size(700, 400);
  num = int((width/xStep)*(height/yStep)); // (Canvas width÷xStep) * (height÷yStep) 
}

 // Function that receives OSC messages from drumsticks
void oscEvent(OscMessage theOscMessage) {
  if (theOscMessage.addrPattern().equals("/gyx")) {
    drawx = theOscMessage.get(0).intValue();
   
  }
  if (theOscMessage.addrPattern().equals("/gyy")) {
    drawy = theOscMessage.get(0).intValue();
    
  }
  if (theOscMessage.addrPattern().equals("/gyz")) {
    drawz = theOscMessage.get(0).intValue();  
  }

}

// Setting the function that will create the graphics in the environment 
void draw(){
  // Color pattern of squares 
  background(drawx, drawy, drawz);
  // Setting the format of the lines 
  strokeCap(CORNER);
  // Setting the width of the lines 
  strokeWeight(xStep); // Line width will equal X axis 
  
  // Setting variable n 
  int n=0;
// As long as "n" is less than "num" 
  while (n<num) {
    // Set color for borders 
    stroke(255-255*cos(radians(a)),255*cos(radians(a)),255-255*sin(radians(a)),255-255*sin(radians(a)));
    // Draw a line 
    line(x, y, x, y+yStep);
    x+=xStep; // Sum the value of x in xStep 
    // If "x" value is greater than width 
    if (x>width) {
      x=xStep/2; // The value of x becomes equivalent to half of xStep 
      y+=yStep; // Y is added to the value of yStep 
    }
    
    // If "Y" is bigger than height 
    if (y>=height) {
      y=0; // The value of y becomes 0 
      a=0; // The value of a becomes 0 
    }
    n++; // Increments 1 in variable n 
    a+=a_; // Add the value of a+ to a_ 
  }
  a_+=0.1; // Add the value of a_ to 0.1 
}
