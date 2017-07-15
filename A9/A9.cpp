#include "A9.hpp"
#include <iostream>

A9::A9()
{
  std::cout << "A9 constructor" << std::endl;

  /*Open serial port to M4*/
  serialport_M4.setSerialPort("/dev/ttyMCC",115200);
  serialport_M4.AddObserver(*this);
  serialport_M4.start();
  destination_set = false;
  
/*Open serial port to gps (does not work.. The seraial port seams to be able
  /* to send data but not receive)*/
  //serialport_GPS.setSerialPort("/dev/ttymxc5",9600);
  //serialport_GPS.AddObserver(*this);
  //serialport_GPS.start();
  //serialport_GPS.send("$PMTK220,5000*1B\r\n"); // position data every 5s
  //serialport_GPS.send("$PMTK300,10000,0,0,0,0*2C\r\n"); //fix data every 10s
  //serialport_GPS.send("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"); //GPRMC and GGA
  //serialport_GPS.send("$PGCMD,33,0*6D\r\n"); //don't send antenna message

  network.AddObserver(*this);
  network.start(5555);

  //first = false;
}
A9::~A9()
{
  std::cout << "A9 destructor" << std::endl;
}

void A9::update()
{
  if(serialport_M4.hasChanged())
  {
    std::string s = serialport_M4.getLine();
    std::cout << "From M4: " << s << std::endl;
    //forward message to pc
    if(destination_set) 
    {
      char* cstr = &s[0u]; //Will this be a memory leak? Or will the memoty be cleared when the string goes out of scope..?
      network.send(cstr,s.length());
      //free(cstr);
    }
  }
  if(serialport_GPS.hasChanged())
  {
    std::cout << "From gps:" << serialport_GPS.getLine() << std::endl;
    gps.decodeMessage(serialport_GPS.getLine());
  }
  if(network.hasChanged())
  {

    std::string s = network.getMessage();
    destination_set = true;
    std::cout << "network message received: " << '"' << s << '"' << std::endl;
    //std::cout << "substr: " << s.substr(0,2) << std::endl;
    if(s.substr(0,2) == ":1")
    {
      //Message is ment for A9
    }
    else if(s.substr(0,2) == ":2")
    {
      //Message is ment for M4
      //std::cout << "message to M4" << std::endl;

      if(s[s.length()-1] == ';') //message is probably OK
      {
        std::string toSend = s.substr(2,s.length()-3);
        //std::cout << "message to send: " << toSend << std::endl;
        serialport_M4.send(toSend);
      }
    }

    /*
    std::cout << "From Android: " << s << std::endl;
    std::string toSend = "$QCPUL";
    toSend += s.substr (10,20);
    toSend += ",*00\n";
    std::cout << "toSend: " << toSend << std::endl;
    serialport_M4.send(toSend);
    */
    /*
    std::cout << "From Android: " << s << std::endl;
    std::string toSend = "$QCSTA,0,0,0,";
    toSend += s.substr (12,3);
    toSend += ",*00\n";
    std::cout << "toSend: " << toSend << std::endl;
    serialport_M4.send(toSend);
    */


    //serialport_M4.send("$QCPUL,1950,1950,1950,1950,*00\n");

    //serialport_M4.send("$QCSTA,0.111,0.222,0.333,100,*00\n");
  }
}
