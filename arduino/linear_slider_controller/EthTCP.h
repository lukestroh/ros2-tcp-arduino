/*
    Header file for custom ethernet class
    Author: Luke Strohbehn
    Date: 4/17/2023

    Robots' ip addresses: 
        Joe's UR5 ip address: 169.254.174.50 
        Cindy's UR5 ip address: 169.254.177.232

    Host pc's ip addresses: 
        Desktop : 169.254.133.21 
        Laptop : 169.254.177.231
*/

#ifndef ETHTCP_H
#define ETHTCP_H

#ifndef DEBUG
#define DEBUG 0
#endif

#include <stdint.h>
#include <Ethernet.h>
#include <Arduino.h>

class Eth {
    private:
        // Connection parameters
        uint8_t MAC[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
        uint8_t IP[4] = { 169, 254, 93, 101 };
        uint8_t DNS[4] = { 169, 254, 93, 234 };
        uint8_t GATEWAY[4] = { 169, 254, 93, 234 };
        uint8_t SUBNET[4] = { 255, 255, 0, 0 };

        

        // Server address, port, client
        IPAddress lan_server_ip;
        const int lan_server_port;
        const int local_server_port;
        EthernetClient local_client;
        EthernetServer local_server;
        int num_clients {1};
        EthernetClient lan_clients[1];

        // Private Methods
    

    public:
        // Data buffer
        int numChars = 32;
        char receivedChars[32];
        bool newData = false;
        
        // Class declaration
        Eth();

        // Public Methods
        void begin_ethernet();
        void begin_server();
        bool connect_local_client();

        // Server
        void accept_clients();
        void remove_clients();
        void read_data();

        // Client
        void send_data(char* data);
        void send_data(float f);

};

#endif // ETHTCP_H
