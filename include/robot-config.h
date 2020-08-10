using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor RightFront;
extern motor RightBack;
extern motor LeftFront;
extern motor LeftBack;
extern controller Controller1;
extern motor MotorExample;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );