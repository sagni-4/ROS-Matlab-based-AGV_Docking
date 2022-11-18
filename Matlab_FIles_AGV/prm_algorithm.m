
bwImage = imread ('robothesis_final.pgm');
bwImage = bwImage <100; 
map = binaryOccupancyMap (bwImage, 100); 
inflate (map, 0.2) ;
prmShowMap = mobileRobotPRM (map, 100) ;
show (prmShowMap) ;