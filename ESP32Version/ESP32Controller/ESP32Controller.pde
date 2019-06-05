/*
 * File: esp32Bot
 * --------------
 * Erick Blankenberg, Beck Goodloe, Josiah Clark
 * ME210
 * 5/26/2019
 * Final
 *
 * Description:
 *   Teleoperated robot that streams lidar data and is controlled
 *   remotely.
 * Notes:
 *   - Make sure to connect to the UDP hotspot before starting
 */

//import oscP5.*;
//import netP5.*;
import hypermedia.net.*;

//OscP5 osc;
//NetAddress esp32;
UDP Esp32Bot; 
boolean pressed = true;
 
 void setup() {
   
   /*
   osc = new OscP5(this, 12000);
   esp32 = new NetAddress("11.11.11.11", 8888);
   size(100, 100);
   */
   
   Esp32Bot = new UDP(this, 12000);
   Esp32Bot.listen(true);
   println("ready");
}

void draw() {
  /*
  if (mousePressed) {
      OscMessage msg = new OscMessage("/led").add((pressed)?1:0);
      osc.send(msg, esp32);
      pressed = !pressed;
      println("Pressed");
      delay(250);
  }
  */
}

void keyPressed() {
    
    String message  = str( key );  // the message to send
    String ip       = "11.11.11.11";  // the remote IP address
    int port        = 8888;    // the destination port
    
    // formats the message for Pd
    message = message+";\n";
    // send the message
    Esp32Bot.send( message, ip, port );
    println("Sent: " + message);
    
}

void recieve(byte[] data) {
  print("Recieved: ");
  println(new String(data));
}
