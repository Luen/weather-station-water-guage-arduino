#include <SPI.h>
#include "epd2in9_V2.h"
#include "epdpaint.h"
#include "fonts.h" // Ensure you have this for font references

#define COLORED 0
#define UNCOLORED 1

unsigned char image[4736]; // for 2.9" display, buffer size must be: display width * display height / 8
Paint paint(image, 0, 0);  // width should be the multiple of 8
Epd epd;

void setup()
{
    Serial.begin(115200);
    if (epd.Init() != 0)
    {
        Serial.print("e-Paper init failed");
        return;
    }

    epd.ClearFrameMemory(0xFF); // bit set = white, bit reset = black
    epd.DisplayFrame();

    paint.SetRotate(ROTATE_0);
    paint.SetWidth(296);  // Set the width to match the display width
    paint.SetHeight(128); // Set the height to match a portion of the display height for modular design

    // Clear the image
    paint.Clear(UNCOLORED);

    // Example for drawing current weather information
    drawCurrentWeather();

    // Example for drawing forecast information
    drawForecast();

    // You must call DisplayFrame() after drawing to update the display
    epd.DisplayFrame();

    // Enter deep sleep mode to conserve power until the next update cycle
    epd.Sleep();
}

void drawCurrentWeather()
{
    // Assuming you have functions to get weather data
    // float temperature = getCurrentTemperature();
    // float humidity = getCurrentHumidity();

    // Current Weather Drawing
    paint.SetWidth(296); // Full width
    paint.SetHeight(64); // Half height for current weather
    paint.Clear(UNCOLORED);
    paint.DrawStringAt(0, 0, "Current Weather", &Font16, COLORED);
    paint.DrawStringAt(0, 20, "Temp: 24°C, Humidity: 40%", &Font12, COLORED);
    epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
}

void drawForecast()
{
    // Forecast Drawing
    paint.SetWidth(296); // Full width
    paint.SetHeight(64); // Another half for forecast
    paint.Clear(UNCOLORED);
    paint.DrawStringAt(0, 0, "Forecast: Sunny", &Font16, COLORED);
    paint.DrawStringAt(0, 20, "Tomorrow: Rain, 22°C", &Font12, COLORED);
    // Adjust Y position based on your layout. Here, it's placed after the current weather section.
    epd.SetFrameMemory(paint.GetImage(), 0, 64, paint.GetWidth(), paint.GetHeight());
}

void loop()
{
    // Leave empty if you're only updating the display on a fixed schedule and using deep sleep in between
}
