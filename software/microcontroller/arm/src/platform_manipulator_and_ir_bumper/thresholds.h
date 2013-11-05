unsigned int input1Channel = 0;

volatile unsigned int link1upperThreshold = 0;
volatile unsigned int link1lowerThreshold = 1023;
volatile unsigned int link1Initial = 1023;
volatile unsigned int link1Range = 1023;

volatile unsigned int link2upperThreshold = 135;
volatile unsigned int link2lowerThreshold = 930;
volatile unsigned int link2Initial = 135;
volatile unsigned int link2Range = 795;

volatile unsigned int link3upperThreshold = 940;
volatile unsigned int link3lowerThreshold = 0;
volatile unsigned int link3Initial = 730;

volatile unsigned int gripperRotateCWThreshold = 167;
volatile unsigned int gripperRotateCWWThreshold = 875;
volatile unsigned int gripperInitialRotation = 0;
volatile unsigned int gripperRotateRange = 708;

volatile unsigned int gripperOpenedThreshold = 180;
volatile unsigned int gripperClosedThreshold = 795;

volatile unsigned int gripperContactThreshold = 800;

volatile unsigned int gripperObjectDetectedThreshold = 500;
