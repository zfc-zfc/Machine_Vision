# Machine-Vision
These are codes for Machine Vision projects.
## Mean Filter
Program to realize the filtering of the image by the mean filter. The filter size is 3x3, 5x5, and two ways are required:
 1. Calculate according to the definition of mean filter;
 2. Use the idea of separating filter to calculate, (Interested students are encouraged to use recursive method to achieve), obtain the filter calculation time, and compare the effects and efficiency of the two filter calculation methods.
 
## Circle Extraction
Calculate the position, area, and approximate diameter of the circular object in the given figure.

## NCC Template Matching
Program to achieve a one-layer NCC template matching algorithm.

## 1D Eage Detection
Use 1D edge extraction method to calculate sub-pixel edge points, try to use circle to calculate the center coordinates and diameter of the circle.
Steps:
1. Roughly determine the position of the center of the circle through image segmentation;
2. Use the approximate center of the circle to generate a ring (the inner and outer radius can be set manually);
3. Generate scan lines every certain angle between the inner and outer circles of the ring (you can set it yourself);
4. Calculate the sub-pixel edge point coordinates on the scan line;
5. Use the circle fitting method to fit the circle equation and give the center position and diameter of the circle.

## Final Project: NCC template matching combining with 8-layer pyramid
