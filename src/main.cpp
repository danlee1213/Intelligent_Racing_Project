/*
 * main.cpp
 *
 * Template Author: LeeChunHei
 *
 */

#include <math.h>

#include "system/system.h"
#include "system/systick.h"

#include "device_driver/MT9V034.h"
#include "device_driver/lcd.h"
#include "device_driver/lcd_console.h"
#include "device_driver/xpt2046.h"
#include "device_driver/touchscreen_lcd.h"
#include "device_driver/st7735r.h"
#include "device_driver/servo.h"


int main() {
	//Init
	System::InitSystem();

	//LCD
    DeviceDriver::St7735r::Config lcdConfig;
	lcdConfig.id = 0;
	lcdConfig.orientation = 1;
	DeviceDriver::St7735r lcd(lcdConfig);

	const DeviceDriver::Lcd::Rect testrect = {0, 0, lcd.GetLcdWidth(), round(120 * lcd.GetLcdWidth() / 184)};
	const DeviceDriver::Lcd::Rect debugrect = {0, round(120 * lcd.GetLcdWidth() / 184), lcd.GetLcdWidth(), lcd.GetLcdHeight() - round(120 * lcd.GetLcdWidth() / 184)};

	//Touch Scren
	DeviceDriver::XPT2046::Config touchConfig;
	touchConfig.id = 0; //lcdConfig.id;
	touchConfig.orientation = 1; //lcdConfig.orientation;
	touchConfig.lcdHeight = (uint32_t) lcd.GetLcdHeight();
	touchConfig.lcdWidth = (uint32_t) lcd.GetLcdWidth();
	
	DeviceDriver::XPT2046 touch(touchConfig);
	uint32_t out_x = 0, out_y = 0;
	bool isSobel = false;

	//Camera
	DeviceDriver::MT9V034::Config cameraConfig;
    cameraConfig.id = 0;
    cameraConfig.width = lcd.GetLcdWidth(); //default 184
    cameraConfig.height = cameraConfig.height = round(120 * lcd.GetLcdWidth() / 184); //default 120
    DeviceDriver::MT9V034 camera(cameraConfig);

    camera.Start();
    uint16_t imgWidth = camera.GetWidth();
    uint16_t imgHeight = camera.GetHeight();
    const uint8_t* cameraDataPtr; // create pointer to recieve buffer from camera

	//Sobel Filter Kernel
	uint8_t sobel[imgHeight][imgWidth] = {0};
	int8_t Gx[] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
    int8_t Gy[] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};

	int centreX = imgWidth/2;
	//Bottom line
	int leftX, rightX;
	int leftThirdX, rightThirdX;
	float leftPercentage = 0.34, rightPercentage = 0.66;
	//Left and Right third
	int leftY, rightY, differenceIndex, differenceSquaredIndex;

	//Servo and Motor
	DeviceDriver::Servo::Config servoConfig;
    servoConfig.id = 0;
    servoConfig.frequency = 50;
    servoConfig.degree = 900;
    servoConfig.min_pos_width = 1000;
    servoConfig.max_pos_width = 2000;
    DeviceDriver::Servo servo(servoConfig);

	DeviceDriver::Servo::Config motorConfig;
    motorConfig.id = 1;
    motorConfig.frequency = 50;
    motorConfig.degree = 900;
    motorConfig.min_pos_width = 1000;
    motorConfig.max_pos_width = 2000;
    DeviceDriver::Servo motor(motorConfig);

	//Thresholds
	const uint16_t leftMax = 600, rightMax = 650, speedMax = 995; //small batt: 985; large: 979


	//Looping
	while(1){
		lcd.SetRegion(testrect);
		if (touch.isTouched(out_x, out_y)) {
			isSobel = !isSobel;
			lcd.FillColor(lcd.kCyan);
		}
		lcd.SetRegion(debugrect);
		lcd.FillColor(lcd.kBlack);
		lcd.SetRegion(testrect);
		if (camera.IsNewFrame()) {
        	cameraDataPtr = camera.LockBuffer();  // Get new frame from camera
			
			//Sobel Filter
			for (int y = 1; y < imgHeight-1; y++) {
				for (int x = 1; x < imgWidth-1; x++) {
					int k = 0;
					uint8_t window[9] = {0};
					int16_t gx = 0;
					int16_t gy = 0;
					double g = 0;
					for (int u=x-1; u<=x+1; u++) {
						for (int v=y-1; v<=y+1; v++) {
							window[k++] = *(cameraDataPtr+imgWidth*v+u);
						}
					}
					for(uint8_t i=0; i<9; i++) {
						gx += Gx[i]*window[i];
					}
					for(uint8_t i=0; i<9; i++) {
						gy += Gy[i]*window[i];
					}
					g = sqrt(gx*gx+gy*gy);
					if (g<150) {
						g = 0;
					} else if (g>200) {
						g = 255;
					}

					sobel[y][x] = g;
				}
			}
			//Sobel bottom line
			/*for (int x = 1; x < imgWidth-1; x++) {
				int k = 0;
				uint8_t window[9] = {0};
				int16_t gx = 0;
				int16_t gy = 0;
				double g = 0;
				for (int u=x-1; u<=x+1; u++) {
					for (int v=imgHeight-1-1; v<=imgHeight-1+1; v++) {
						window[k++] = *(cameraDataPtr+imgWidth*v+u);
					}
				}
				for(uint8_t i=0; i<9; i++) {
					gx += Gx[i]*window[i];
				}
				for(uint8_t i=0; i<9; i++) {
					gy += Gy[i]*window[i];
				}
				g = sqrt(gx*gx+gy*gy);
				if (g<150) {
					g = 0;
				} else if (g>200) {
					g = 255;
				}

				sobel[imgHeight - 2][x] = g;
			}*/

			//Get bottom left and right edges' x-coord
			leftX = centreX, rightX = centreX;
			while (sobel[imgHeight - 1][leftX] != 255 && leftX > 1) {
				leftX--;
			}
			while (sobel[imgHeight - 1][rightX] != 255 && rightX < imgWidth - 2) {
				rightX++;
			}
			leftThirdX = round((rightX - leftX) * leftPercentage);
			rightThirdX = round((rightX - leftX) * rightPercentage);

			//Sobel two vertical dividers
			/*for (int y = 1; y < imgHeight-1; y++) {
				int k; uint8_t window[9]; int16_t gx, gy; double g;
				//left one
				k = 0;
				window[9] = {0};
				gx = 0;
				gy = 0;
				g = 0;
				for (int u=leftThirdX-1; u<=leftThirdX+1; u++) {
					for (int v=y-1; v<=y+1; v++) {
						window[k++] = *(cameraDataPtr+imgWidth*v+u);
					}
				}
				for(uint8_t i=0; i<9; i++) {
					gx += Gx[i]*window[i];
				}
				for(uint8_t i=0; i<9; i++) {
					gy += Gy[i]*window[i];
				}
				g = sqrt(gx*gx+gy*gy);
				if (g<150) {
					g = 0;
				} else if (g>200) {
					g = 255;
				}

				sobel[y][leftThirdX] = g;
				//right one
				k = 0;
				window[9] = {0};
				gx = 0;
				gy = 0;
				g = 0;
				for (int u=rightThirdX-1; u<=rightThirdX+1; u++) {
					for (int v=y-1; v<=y+1; v++) {
						window[k++] = *(cameraDataPtr+imgWidth*v+u);
					}
				}
				for(uint8_t i=0; i<9; i++) {
					gx += Gx[i]*window[i];
				}
				for(uint8_t i=0; i<9; i++) {
					gy += Gy[i]*window[i];
				}
				g = sqrt(gx*gx+gy*gy);
				if (g<150) {
					g = 0;
				} else if (g>200) {
					g = 255;
				}

				sobel[y][rightThirdX] = g;
			}*/

			
			//Calculate indices for turning
			int leftY = imgHeight - 1, rightY = imgHeight - 1;
			int tempLeftThirdX = leftThirdX, tempRightThirdX = rightThirdX;
			while (sobel[leftY][leftThirdX] != 255 && leftY > 1) {
				leftY--;
				tempLeftThirdX++;
				if (tempLeftThirdX >= leftThirdX + 25) {
					leftThirdX++;
					tempLeftThirdX = leftThirdX;
				}
			}
			while (sobel[rightY][rightThirdX] != 255 && rightY > 1) {
				rightY--;
				tempRightThirdX--;
				if (tempRightThirdX <= rightThirdX - 25) {
					rightThirdX--;
					tempRightThirdX = rightThirdX;
				}
			}
			differenceIndex = round((rightY - leftY) / 2);
			differenceSquaredIndex = round((pow(rightY, 2) - pow(leftY, 2)) * 3);

			
			//Motor and Servo
			if (differenceSquaredIndex < -leftMax) {
				differenceSquaredIndex = -leftMax;
			} else if (differenceSquaredIndex > rightMax) {
				differenceSquaredIndex = rightMax;
			}
			servo.SetDegree(900 + differenceSquaredIndex);
			motor.SetDegree(speedMax - round(sqrt(pow(differenceIndex, 2)) * 0.58));

			lcd.FillGrayscalePixel(isSobel ? *sobel : cameraDataPtr, lcd.GetLcdWidth() * lcd.GetLcdHeight());
    	}
	}
}
