#!/usr/bin/env python3
"""OLED display test for SSD1306 (128x64)."""
import board
import busio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
# Setup I2C and display
i2c = busio.I2C(board.SCL, board.SDA)
oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c)
# Clear display
oled.fill(0)
oled.show()
# Create image buffer (1-bit color)
image = Image.new("1", (oled.width, oled.height))
draw = ImageDraw.Draw(image)
# Load a font (default or custom)
try:
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)
    font_small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 10)
except OSError:
    font = ImageFont.load_default()
    font_small = font
# Draw content
draw.rectangle((0, 0, 127, 63), outline=1, fill=0)  # Border
draw.text((10, 5), "RoboTrail", font=font, fill=1)
draw.text((10, 22), "ToF: OK", font=font_small, fill=1)
draw.text((10, 34), "IMU: OK", font=font_small, fill=1)
draw.text((10, 46), "Batt: --", font=font_small, fill=1)
# Display it
oled.image(image)
oled.show()
print("OLED test complete - check the display!")