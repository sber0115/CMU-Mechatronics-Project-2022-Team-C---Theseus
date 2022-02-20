import processing.serial.*;
import controlP5.*;

Serial port;

ControlP5 cp5;

PFont font;

Knob servo_knob;
Knob dc_knob;
Textarea servo_label;
Textarea dc_label;
Textarea stepper_label;
Textarea stepper;
Textarea potentiometer_label;
Textarea potentiometer;
Textarea ultrasonic_label;
Textarea ultrasonic;

int stepper_ang;

String[] list;

void setup(){
  size(600, 600); 
  
  port = new Serial(this, "COM7", 115200); 
  
  cp5 = new ControlP5(this);
  
  font = createFont("Arial", 20);
  
  stepper_ang = 0;
  
  servo_knob = cp5.addKnob("servo_knob_fun")
               .setPosition(50,100)
               .setRange(-50,50)
               .setValue(0)
               .setRadius(50)
               .setDragDirection(Knob.VERTICAL)
               ;
  servo_label = cp5.addTextarea("servo_label_textarea")
               .setPosition(42,200)
               .setSize(175,50)
               .setBorderColor(color(0))
               .setColor(color(0))
               .setFont(font)
               ;
  servo_label.setText("Servo RPM");
  
  dc_knob = cp5.addKnob("dc_knob_fun")
               .setPosition(250,100)
               .setRange(-60,60)
               .setValue(0)
               .setRadius(50)
               .setDragDirection(Knob.VERTICAL)
               ;
               
  dc_label = cp5.addTextarea("dc_label_textarea")
               .setPosition(227,200)
               .setSize(175,50)
               .setBorderColor(color(0))
               .setColor(color(0))
               .setFont(font)
               ;
  dc_label.setText("DC Motor RPM");
  
  stepper_label = cp5.addTextarea("stepper_label_textarea")
               .setPosition(425,120)
               .setSize(175,50)
               .setBorderColor(color(0))
               .setColor(color(0))
               .setFont(font)
               ;
  stepper_label.setText("Stepper Angle");
  
  stepper = cp5.addTextarea("stepper_textarea")
               .setPosition(475,150)
               .setSize(50,50)
               .setBorderColor(color(0))
               .setColor(color(0))
               .setFont(font)
               ;
  stepper.setText(Integer.toString(stepper_ang));
  
  potentiometer_label = cp5.addTextarea("potentiometer_label_textarea")
               .setPosition(60,300)
               .setSize(200,50)
               .setBorderColor(color(0))
               .setColor(color(0))
               .setFont(font)
               ;     
  potentiometer_label.setText("Potentiometer Angle");
  
  potentiometer = cp5.addTextarea("potentiometer_textarea")
               .setPosition(135,330)
               .setSize(50,50)
               .setBorderColor(color(0))
               .setColor(color(0))
               .setFont(font)
               ;     
  potentiometer.setText("0");
  
  ultrasonic_label = cp5.addTextarea("ultrasonic_label_textarea")
               .setPosition(350,300)
               .setSize(200,50)
               .setBorderColor(color(0))
               .setColor(color(0))
               .setFont(font)
               ;     
  ultrasonic_label.setText("Ultrasonic Distance");
  
  ultrasonic = cp5.addTextarea("ultrasonic_textarea")
               .setPosition(425,330)
               .setSize(50,50)
               .setBorderColor(color(0))
               .setColor(color(0))
               .setFont(font)
               ;     
  ultrasonic.setText("0");
}

void draw(){
  background(255, 255, 255); 
  fill(0, 0, 0);
  text("MOTOR LAB", 250, 20); 
  
  while (port.available() > 0) {
    String str = port.readStringUntil('\n');
    if(str != null){
      //println(str);
      str = trim(str);
      list = split(str, ':');
      if(list.length == 2){
        String type = list[0];
        String val = list[1];
        switch (type) {
           case "potentiometer":
             potentiometer.setText(val);
           case "ping_sensor":
             ultrasonic.setText(val);
             
        }
      }
    }
  }
  
  
  delay(211);
}

//lets add some functions to our buttons
//so whe you press any button, it sends perticular char over serial port

void servo_knob_fun(int val){
  String str = String.format("servo:%d\n", -val);
  port.write(str);
}

void dc_knob_fun(int val){
  port.write(String.format("dc:%d\n", val));
}
