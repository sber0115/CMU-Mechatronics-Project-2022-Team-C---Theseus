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
Textfield stepper;
Textarea potentiometer_label;
Textarea potentiometer;
Textarea ultrasonic_label;
Textarea ultrasonic;
Textarea manual_text;
Textarea sensor_text;

int stepper_ang;

String[] list;
String stepper_ang_txt;

Boolean manual = false;

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
  
  stepper = cp5.addTextfield("stepper_input")
               .setFont(font)
               .setPosition(450, 150)
               .setSize(50, 50)
               .setAutoClear(false)
               ;
  stepper.setText(Integer.toString(stepper_ang));
  
  cp5.addBang("submit_stepper").setPosition(505, 150).setSize(30, 30);
  
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
  
  cp5.addToggle("manual_toggle")
     .setPosition(250,410)
     .setSize(50,20)
     .setValue(true)
     .setMode(ControlP5.SWITCH)
     ;
     
  manual_text = cp5.addTextarea("manual_textarea")
               .setPosition(163,400)
               .setSize(100,50)
               .setBorderColor(color(0))
               .setColor(color(0))
               .setFont(font)
               .setText("Manual")
               ;
               
  sensor_text = cp5.addTextarea("sensor_textarea")
               .setPosition(315,400)
               .setSize(100,50)
               .setBorderColor(color(0))
               .setColor(color(0))
               .setFont(font)
               .setText("Sensor")
               ;
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
           case "ultrasonic":
             ultrasonic.setText(val);
           //case "force":
           //  break;
        }
      }
    }
  }
  
  
  delay(211);
}

//lets add some functions to our buttons
//so whe you press any button, it sends perticular char over serial port

void servo_knob_fun(int val){
  port.write(String.format("servo:%d\n", -val));
}

void dc_knob_fun(int val){
  port.write(String.format("dc:%d\n", val));
}

void submit_stepper(){
  stepper_ang = Integer.valueOf(cp5.get(Textfield.class,"stepper_input").getText());
  port.write(String.format("stepper:%d\n", stepper_ang));
}

void manual_toggle(){
  manual = !manual;
  port.write(String.format("manual:%b\n", manual));
}
