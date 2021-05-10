# Intelligent_Racing_Project
Intelligent Racing Project for [NXP Cup National University Students Intelligent Car Race](https://community.nxp.com/t5/The-NXP-Cup-EMEA/gh-p/15351)

## Overview
This is the guiding algorithm from HKUST Robotics Team (Smart Car Subteam) for NXP Cup National University Students Intellignet Car Race.

## Race Track Schematic
![Track](https://user-images.githubusercontent.com/72503871/112940665-09aaab80-9160-11eb-8132-6a2d238be2be.jpg)

It is really important to handle the car movement at the intersection on the middle of the track. 
The algorithm must ensure that the car is automatically guided straightfoward at the intersection, not to move left or right.

## Hardware
i.MX RT1052 Crossover MCU with ARM Cortex-M7 Core has been used.

[Technical Data i.MX RT1050](https://www.nxp.com/docs/en/data-sheet/IMXRT1050CEC.pdf)

[Reference Manual](https://drive.google.com/file/d/1-84KOASvyucwciEcXKLrr0VAlIWJbvv-/view?usp=sharing)

## Software

### Applying sobel filter on Camera pointer images

#### 1. Right corner
<div>
  <img src=https://user-images.githubusercontent.com/72503871/117583612-acc8ea80-b13a-11eb-806e-819e91ac280f.jpg width="430">
  <img src=https://user-images.githubusercontent.com/72503871/117583590-886d0e00-b13a-11eb-8496-60d036c0fd65.jpg width="430" align="right">
</div>

#### 2. Left corner
<div>
  <img src=https://user-images.githubusercontent.com/72503871/117584754-3aa7d400-b141-11eb-938b-c9632a52fc7e.jpg width="430">
  <img src=https://user-images.githubusercontent.com/72503871/117584750-38de1080-b141-11eb-9671-33c03b1dc788.jpg width="430" align="right">
</div>

#### 3. Crossroad
<div>
  <img src=https://user-images.githubusercontent.com/72503871/117584842-ca4d8280-b141-11eb-9cd6-57859e0bc674.jpg width="430">
  <img src=https://user-images.githubusercontent.com/72503871/117584844-cc174600-b141-11eb-8c3f-bfcc769f04da.jpg width="430" align="right">
</div>


Sobel operator uses 3x3 kernels which are convolved with the original image to calculate approximations of the derivatives (One for horizontal and one for vertical changes):
```C++
//Sobel Filter Kernel
uint8_t sobel[imgHeight][imgWidth] = {0};
int8_t Gx[] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
int8_t Gy[] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
```

Apply sobel filter as follows:
```C++
//Sobel Filter
for(int y=1; y<imgHeight-1; y++){
    for(int x=1; x<imgWidth-1; x++){
        int k = 0;
        uint8_t window[9] = 0;
        int16_t gx = 0;
        int16_t gy = 0;
        double g = 0;
        for(int u=x-1; u<=x+1; u++){
            for(int v=y-1; v<=y+1; v++){
                window[k++] = *(cameraDataPtr+imgWidth*v+u);
            }
        }
        for(uint8_t i=0; i<9; i++){
            gx += Gx[i]*window[i];
        }
        for(uint8_t i=0; i<9; i++){
            gy += Gy[i]*window[i];
        }
        g = sqrt(gx*gx+gy*gy);
        if(g < 150){
            g = 0; //Black
        } else if(g > 200){
            g = 255; //White
        }
        sobel[y][x] = g;
    }
}
```

### How to recognize left, right corner and crossroad?

                                                                                                                      
