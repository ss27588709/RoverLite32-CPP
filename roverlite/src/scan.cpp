#include <Wire.h>
#include <Arduino.h>
#include <camera_pins.h>

void setup()
{
    Serial.begin(115200);
    Serial.println();
    Serial.println("Scanning for I2C Devices ...");
    Serial.print("\r\n");
    int I2CDevices = 0;
    u_int8_t _i2caddr = 0x40;
    Wire.setPins(I2C_SDA, I2C_SCL);
    Wire.begin();
    for (int i = 0; i < 5; i++)
    {
        Wire.beginTransmission(0x70);
        Wire.write(0x00);
        Wire.write(0x05);
        if (!Wire.endTransmission())
            break; // if no error, break out of for loop
        Serial.println("write operation failed");
    }
    while (Wire.available())
    {
        uint8_t data = Wire.read();
        Serial.printf("kjjjj%d", data);
    }

    for (byte address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0)
        {
            Serial.print("Found I2C Device: ");
            Serial.print(" (0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.print(address, HEX);
            Serial.println(")");
            I2CDevices++;
        }
    }
    if (I2CDevices == 0)
    {
        Serial.println("No device!\n");
    }
    else
    {
        Serial.print("get it");
        Serial.print(I2CDevices);
        Serial.println("number devices!\n");
    }
}

void loop()
{
}