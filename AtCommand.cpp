#include "XBee.h"

using namespace std;

XBee xbee = XBee();

// serial high
uint8_t shCmd[] = {'S','H'};
// serial low
uint8_t slCmd[] = {'S','L'};
// association status
uint8_t assocCmd[] = {'A','I'};

AtCommandRequest atRequest = AtCommandRequest(shCmd);

AtCommandResponse atResponse = AtCommandResponse();

void sendAtCommand() {
  cout << "Sending command to the XBee" << endl;

  // send the command
  xbee.send(atRequest);

  // wait up to 5 seconds for the status response
  if (xbee.readPacket(5000)) {
    // got a response!

    // should be an AT command response
    if (xbee.getResponse().getApiId() == AT_COMMAND_RESPONSE) {
      xbee.getResponse().getAtCommandResponse(atResponse);

      if (atResponse.isOk()) {
        cout << "Command [";
        cout << atResponse.getCommand()[0];
        cout << atResponse.getCommand()[1];
        cout << "] was successful!" << endl;

        if (atResponse.getValueLength() > 0) {
          cout << "Command value length is ";
          cout << dec << (int)atResponse.getValueLength();
		  cout << endl;
		  
          cout << "Command value: ";
          
          for (int i = 0; i < atResponse.getValueLength(); i++) {
            cout << hex << (int)atResponse.getValue()[i];
			cout << " ";
          }

          cout << endl;
        }
      } 
      else {
        cout << "Command return error code: ";
        cout << hex << (int)atResponse.getStatus();
		cout << endl;
      }
    } else {
      cout << "Expected AT response but got ";
      cout << hex << (int)xbee.getResponse().getApiId();
    }   
  } else {
    // at command failed
    if (xbee.getResponse().isError()) {
      cout << "Error reading packet.  Error code: ";  
      cout << dec << (int)xbee.getResponse().getErrorCode();
	  cout << endl;
    } 
    else {
      cout << "No response from radio";  
    }
  }
}


int main() {
  Serial.begin("/dev/ttyS0", 115200);
  xbee.begin(Serial);
   sendAtCommand();
  
  // set command to SL
  atRequest.setCommand(slCmd);  
  sendAtCommand();

  // set command to AI
  atRequest.setCommand(assocCmd);  
  sendAtCommand();
  
  while (1) { }
}



