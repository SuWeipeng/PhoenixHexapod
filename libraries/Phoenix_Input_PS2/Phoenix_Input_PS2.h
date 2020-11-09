//====================================================================
//Project Lynxmotion Phoenix //Lynxmotion凤凰项目
//Description: Phoenix, control file.
//The control input subroutine for the phoenix software is placed in this file.
//Can be used with V2.0 and above
//Configuration version: V1.0
//Date: 25-10-2009
//Programmer: Jeroen Janssen (aka Xan)
//             Kurt Eckhardt (aka KurtE) - converted to c ported to Arduino...
//
//Hardware setup: PS2 version //硬件设置：PS2版本
// 
//NEW IN V1.0
//- First Release
//
//Walk method 1: 步行方法1：
//- Left StickWalk/Strafe 左拐弯/走路
//- Right StickRotate 右旋转
//
//Walk method 2: 步行方法2：
//- Left StickDisable 左键禁用
//- Right StickWalk/Rotate Right 右拐弯/旋转
//
//
//PS2 CONTROLS: PS2控制：
//[Common Controls] 公共控制
//- StartTurn on/off the bot 开始打开/关闭
//- L1Toggle Shift mode L1切换Shift模式 
//- L2Toggle Rotate mode L2Toggle旋转模式
//- CircleToggle Single leg mode 圆圈 单腿模式
//   - Square        Toggle Balance mode  - 方形 切换平衡模式
//- TriangleMove body to 35 mm from the ground (walk pos) 三角 将身体移动到离地面35毫米处（步行pos）
//and back to the ground 并返回到地面
//- D-Pad upBody up 10 mm D-Pad up身体达10毫米
//- D-Pad downBody down 10 mm
//- D-Pad leftdecrease speed with 50mS D-Pad降低了50mS的速度
//- D-Pad rightincrease speed with 50mS
//
// Optional: L3 button down, Left stick can adjust leg positions... 
//可选：L3按钮向下，左侧杆可以调整腿部位置...
// or if OPT_SINGLELEG is not defined may try using Circle
//或者如果OPT_SINGLELEG未定义，可以尝试使用Circle
//
//
//[Walk Controls]  // [步行控制]
//- selectSwitch gaits // - 选择开关步态
//- Left Stick(Walk mode 1) Walk/Strafe  // - 左棒（步行模式1）步行/绞盘
// (Walk mode 2) Disable  //（步行模式2）禁用
//- Right Stick(Walk mode 1) Rotate,  // - 右棒（步行模式1）旋转，
//(Walk mode 2) Walk/Rotate  //（步行模式2）步行/旋转
//- R1Toggle Double gait travel speed  // - R1Toggle双步行走速度
//- R2Toggle Double gait travel length  // - R2Toggle双步行程
//
//[Shift Controls]  // [Shift控制]
//- Left StickShift body X/Z  // - 左杆移位身体X / Z
//- Right StickShift body Y and rotate body Y  // - 右杆移动主体Y并旋转主体Y.
// 
//[Rotate Controls]  // [旋转控制]
//- Left StickRotate body X/Z   // - 左摇杆旋转主体X / Z
//- Right StickRotate body Y    // - 右棒旋转机身Y.
//
//[Single leg Controls]   // [单腿控制]
//- selectSwitch legs   // - 选择切换腿
//- Left StickMove Leg X/Z (relative)  // - 左拐移动腿X / Z（相对）
//- Right StickMove Leg Y (absolute)  // - 右手移动腿Y（绝对）
//- R2Hold/release leg position   // - R2保持/释放腿部位置
//
//[GP Player Controls]  // [GP Player控件]
//- selectSwitch Sequences  // - 选择切换序列
//- R2Start Sequence   // - R2启动序列
//
//====================================================================
// [Include files]
#if ARDUINO>99
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif
#include <PS2X_lib.h>

//[CONSTANTS]
#define WALKMODE          0    //行走模式
#define TRANSLATEMODE     1    //翻译模式
#define ROTATEMODE        2    //旋转模式
#define SINGLELEGMODE     3		//单腿模式 
#define GPPLAYERMODE      4		//


#define cTravelDeadZone 4      //The deadzone for the analog input from the remote  
//来自遥控器的模拟输入的死区
#define  MAXPS2ERRORCNT  5     // How many times through the loop will we go before shutting off robot?
//在关闭机器人之前，我们将通过循环多少次？

#ifndef MAX_BODY_Y
#define MAX_BODY_Y 100
#endif

//=============================================================================
// Global - Local to this file only...  全局 - 仅限本地文件...
//=============================================================================
PS2X ps2x; // create PS2 Controller Class  创建PS2控制器类


// Define an instance of the Input Controller...  定义输入控制器的一个实例...
InputController  g_InputController;       // Our Input controller 我们的输入控制器


static short      g_BodyYOffset; //身体Y偏移
static short      g_sPS2ErrorCnt; //PS2错误计数
static short       g_BodyYShift;   //身体Y转移
static byte        ControlMode;	//控制模式
static bool        DoubleHeightOn;	//双高度上
static bool        DoubleTravelOn;	//旅行
static bool        WalkMethod;	//步行法
byte            GPSeq;             //Number of the sequence 序列号
short              g_sGPSMController;    // What GPSM value have we calculated. 0xff - Not used yet
//我们计算了什么GPSM值。 0xff - 尚未使用

// some external or forward function references.	一些外部或前向函数引用。
extern void PS2TurnRobotOff(void); //关闭机器人

//==============================================================================
// This is The function that is called by the Main program to initialize
//the input controller, which in this case is the PS2 controller
//process any commands.
//这是主程序调用的初始化函数
//输入控制器，在这种情况下是PS2控制器
//处理任何命令。

//==============================================================================

// If both PS2 and XBee are defined then we will become secondary to the xbee
//如果同时定义了PS2和XBee，那么我们将会成为xbee的继承者
void InputController::Init(void)
{
  int error;

  //error = ps2x.config_gamepad(57, 55, 56, 54);  // Setup gamepad (clock, command, attention, data) pins
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT);  // Setup gamepad (clock, command, attention, data) pins
//安装游戏手柄（时钟，命令，注意，数据）引脚
#ifdef DBGSerial
	DBGSerial.print("PS2 Init: ");
	DBGSerial.println(error, DEC);
#endif
  g_BodyYOffset = 0;    
  g_BodyYShift = 0;
  g_sPS2ErrorCnt = 0;  // error count

  ControlMode = WALKMODE;
  DoubleHeightOn = false;
  DoubleTravelOn = false;
  WalkMethod = false;

  g_InControlState.SpeedControl = 120;    // Sort of migrate stuff in from Devon.
}

//==============================================================================
// This function is called by the main code to tell us when it is about to
// do a lot of bit-bang outputs and it would like us to minimize any interrupts
// that we do while it is active...
//这个函数被主代码调用来告诉我们什么时候该做什么
//做很多bit-bang输出，它会让我们尽量减少中断
//我们在活动时所做的事情...
//==============================================================================
void InputController::AllowControllerInterrupts(boolean fAllow)
{
  // We don't need to do anything...
}

//==============================================================================
// This is The main code to input function to read inputs from the PS2 and then
//process any commands.
//==============================================================================
#ifdef OPT_DYNAMIC_ADJUST_LEGS  
boolean g_fDynamicLegXZLength = false;  // Has the user dynamically adjusted the Leg XZ init pos (width)
#endif

void InputController::ControlInput(void)
{
  boolean fAdjustLegPositions = false;
  // Then try to receive a packet of information from the PS2.
  // Then try to receive a packet of information from the PS2.
  ps2x.read_gamepad();          //read controller and set large motor to spin at 'vibrate' speed

    // Wish the library had a valid way to verify that the read_gamepad succeeded... Will hack for now
  if ((ps2x.Analog(1) & 0xf0) == 0x70) {
#ifdef DBGSerial
#ifdef DEBUG_PS2_INPUT
	if (g_fDebugOutput) {
		DBGSerial.print("PS2 Input: ");
		DBGSerial.print(ps2x.ButtonDataByte(), HEX);
		DBGSerial.print(":");
		DBGSerial.print(ps2x.Analog(PSS_LX), DEC);
		DBGSerial.print(" ");
		DBGSerial.print(ps2x.Analog(PSS_LY), DEC);
		DBGSerial.print(" ");
		DBGSerial.print(ps2x.Analog(PSS_RX), DEC);
		DBGSerial.print(" ");
		DBGSerial.println(ps2x.Analog(PSS_RY), DEC);
	}
#endif
#endif

#ifdef OPT_DYNAMIC_ADJUST_LEGS  
    boolean fAdjustLegPositions = false;
    short sLegInitXZAdjust = 0;
    short sLegInitAngleAdjust = 0;
#endif
    // In an analog mode so should be OK...
    g_sPS2ErrorCnt = 0;    // clear out error count...

    if (ps2x.ButtonPressed(PSB_START)) {// OK lets press start button
      if (g_InControlState.fRobotOn) {
        PS2TurnRobotOff();
      } 
      else {
        //Turn on
        g_InControlState.fRobotOn = 1;
        fAdjustLegPositions = true;
      }
    }

    if (g_InControlState.fRobotOn) {
      // [SWITCH MODES]

      //Translate mode
      if (ps2x.ButtonPressed(PSB_L1)) {// L1 Button Test
        MSound( 1, 50, 2000);  
        if (ControlMode != TRANSLATEMODE )
          ControlMode = TRANSLATEMODE;
        else {
#ifdef OPT_SINGLELEG
          if (g_InControlState.SelectedLeg==255) 
            ControlMode = WALKMODE;
          else
#endif
            ControlMode = SINGLELEGMODE;
        }
      }

      //Rotate mode
      if (ps2x.ButtonPressed(PSB_L2)) {    // L2 Button Test
        MSound( 1, 50, 2000);
        if (ControlMode != ROTATEMODE)
          ControlMode = ROTATEMODE;
        else {
#ifdef OPT_SINGLELEG
          if (g_InControlState.SelectedLeg == 255) 
            ControlMode = WALKMODE;
          else
#endif
            ControlMode = SINGLELEGMODE;
        }
      }

      //Single leg mode fNO
#ifdef OPT_SINGLELEG

      if (ps2x.ButtonPressed(PSB_CIRCLE)) {// O - Circle Button Test
	   MSound(1, 50, 2000);  

	  SSCSerial.println("Line#1P412T1000");
        if (abs(g_InControlState.TravelLength.x)<cTravelDeadZone && abs(g_InControlState.TravelLength.z)<cTravelDeadZone 
          && abs(g_InControlState.TravelLength.y*2)<cTravelDeadZone )   {
          if (ControlMode != SINGLELEGMODE) {
            ControlMode = SINGLELEGMODE;
            if (g_InControlState.SelectedLeg == 255)  //Select leg if none is selected
              g_InControlState.SelectedLeg=cRF; //Startleg
          } 
          else {
            ControlMode = WALKMODE;
            g_InControlState.SelectedLeg=255;
          }
        }
      }      
#endif
#ifdef OPT_GPPLAYER
int POS;
int Contt;
      if (ps2x.ButtonPressed(PSB_CIRCLE)) {// O - Circle Button Test
	   MSound(1, 50, 2000);  
	   
	   Contt=Contt++;
             if(Contt>1)
			{
              Contt=0;
           }
           if(Contt==0)
           {
			POS=212; 
		SSCSerial.print("Line#1P");
		  SSCSerial.print(POS, DEC);
		  SSCSerial.println("T500");
			//Contt=0;
           }
         if(Contt==1)
           {   
 		  POS=412;
		SSCSerial.print("Line#1P");
		  SSCSerial.print(POS, DEC);
		  SSCSerial.println("T500");
			//Contt=1;         

           }
	  

      } 
      // GP Player Mode X
	  
      if (ps2x.ButtonPressed(PSB_CROSS)) { // X - Cross Button Test
        MSound(1, 50, 2000);  
		
		
        if (ControlMode != GPPLAYERMODE) {
          ControlMode = GPPLAYERMODE;
          GPSeq=0;
			

        } 
        else
		{
		  
          ControlMode = WALKMODE;
		 
		}
		
      }
#endif // OPT_GPPLAYER

      //[Common functions]
      //Switch Balance mode on/off 
      if (ps2x.ButtonPressed(PSB_SQUARE)) { // Square Button Test
        g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
        if (g_InControlState.BalanceMode) {
          MSound(1, 250, 1500); 
        } 
        else {
          MSound( 2, 100, 2000, 50, 4000);
        }
      }

      //Stand up, sit down  
      if (ps2x.ButtonPressed(PSB_TRIANGLE)) { // Triangle - Button Test
        if (g_BodyYOffset>0) 
          g_BodyYOffset = 0;
        else
          g_BodyYOffset = 35;
        fAdjustLegPositions = true;
      }

      if (ps2x.ButtonPressed(PSB_PAD_UP)) {// D-Up - Button Test
        g_BodyYOffset += 10;

        // And see if the legs should adjust...
        fAdjustLegPositions = true;
        if (g_BodyYOffset > MAX_BODY_Y)
          g_BodyYOffset = MAX_BODY_Y;
      }

      if (ps2x.ButtonPressed(PSB_PAD_DOWN) && g_BodyYOffset) {// D-Down - Button Test
        if (g_BodyYOffset > 10)
          g_BodyYOffset -= 10;
        else
          g_BodyYOffset = 0;      // constrain don't go less than zero.

        // And see if the legs should adjust...
        fAdjustLegPositions = true;
      }

      if (ps2x.ButtonPressed(PSB_PAD_RIGHT)) { // D-Right - Button Test
        if (g_InControlState.SpeedControl>20) {
          g_InControlState.SpeedControl = g_InControlState.SpeedControl - 20;
          MSound( 1, 50, 2000);  
        }
      }

      if (ps2x.ButtonPressed(PSB_PAD_LEFT)) { // D-Left - Button Test
        if (g_InControlState.SpeedControl<1000 ) {
          g_InControlState.SpeedControl = g_InControlState.SpeedControl + 20;
          MSound( 1, 50, 2000); 
        }
      }
      
      // We are optionally going to allow the user to modify the Initial Leg positions, when they
      // press the L3 button.
      byte lx = ps2x.Analog(PSS_LX);
      byte ly = ps2x.Analog(PSS_LY);
#ifdef OPT_DYNAMIC_ADJUST_LEGS  
#ifdef OPT_SINGLELEG
      if (ps2x.Button(PSB_L3)) {    // L3 pressed, use this to modify leg positions.
#else
      if (ps2x.Button(PSB_CIRCLE)) {// O - Circle Button Test 
#endif      
        sLegInitXZAdjust = ((int)lx-128)/10;        // play with this.
        sLegInitAngleAdjust = ((int)ly-128)/8;
        lx = 0;
        ly = 0;
      }
#endif

      //[Walk functions]
      if (ControlMode == WALKMODE) {
        //Switch gates
        if (ps2x.ButtonPressed(PSB_SELECT)            // Select Button Test
        && abs(g_InControlState.TravelLength.x)<cTravelDeadZone //No movement
        && abs(g_InControlState.TravelLength.z)<cTravelDeadZone 
          && abs(g_InControlState.TravelLength.y*2)<cTravelDeadZone  ) {
          g_InControlState.GaitType = g_InControlState.GaitType+1;                    // Go to the next gait...
          if (g_InControlState.GaitType<NUM_GAITS) {                 // Make sure we did not exceed number of gaits...
            MSound( 1, 50, 2000); 
          } 
          else {
            MSound(2, 50, 2000, 50, 2250); 
            g_InControlState.GaitType = 0;
          }
          GaitSelect();
        }

        //Double leg lift height
        if (ps2x.ButtonPressed(PSB_R1)) { // R1 Button Test
          MSound( 1, 50, 2000); 
          DoubleHeightOn = !DoubleHeightOn;
          if (DoubleHeightOn)
            g_InControlState.LegLiftHeight = 80;
          else
            g_InControlState.LegLiftHeight = 50;
        }

        //Double Travel Length
        if (ps2x.ButtonPressed(PSB_R2)) {// R2 Button Test
          MSound(1, 50, 2000); 
          DoubleTravelOn = !DoubleTravelOn;
        }

        // Switch between Walk method 1 && Walk method 2
        if (ps2x.ButtonPressed(PSB_R3)) { // R3 Button Test
          MSound(1, 50, 2000); 
          WalkMethod = !WalkMethod;
        }

        //Walking
        if (WalkMethod)  //(Walk Methode) 
          g_InControlState.TravelLength.z = (ps2x.Analog(PSS_RY)-128); //Right Stick Up/Down  

        else {
          g_InControlState.TravelLength.x = -(lx - 128);
          g_InControlState.TravelLength.z = (ly - 128);
        }

        if (!DoubleTravelOn) {  //(Double travel length)
          g_InControlState.TravelLength.x = g_InControlState.TravelLength.x/2;
          g_InControlState.TravelLength.z = g_InControlState.TravelLength.z/2;
        }

        g_InControlState.TravelLength.y = -(ps2x.Analog(PSS_RX) - 128)/4; //Right Stick Left/Right 
      }

      //[Translate functions]
      g_BodyYShift = 0;
      if (ControlMode == TRANSLATEMODE) {
        g_InControlState.BodyPos.x = (lx - 128)/2;
        g_InControlState.BodyPos.z = -(ly - 128)/3;
        g_InControlState.BodyRot1.y = (ps2x.Analog(PSS_RX) - 128)*2;
        g_BodyYShift = (-(ps2x.Analog(PSS_RY) - 128)/2);
      }

      //[Rotate functions]
      if (ControlMode == ROTATEMODE) {
        g_InControlState.BodyRot1.x = (ly - 128);
        g_InControlState.BodyRot1.y = (ps2x.Analog(PSS_RX) - 128)*2;
        g_InControlState.BodyRot1.z = (lx - 128);
        g_BodyYShift = (-(ps2x.Analog(PSS_RY) - 128)/2);
      }

      //[Single leg functions]
#ifdef OPT_SINGLELEG
      if (ControlMode == SINGLELEGMODE) {
        //Switch leg for single leg control
        if (ps2x.ButtonPressed(PSB_SELECT)) { // Select Button Test
          MSound(1, 50, 2000); 
          if (g_InControlState.SelectedLeg<(CNT_LEGS-1))
            g_InControlState.SelectedLeg = g_InControlState.SelectedLeg+1;
          else
            g_InControlState.SelectedLeg=0;
        }

        g_InControlState.SLLeg.x= (lx - 128)/2; //Left Stick Right/Left
        g_InControlState.SLLeg.y= (ps2x.Analog(PSS_RY) - 128)/10; //Right Stick Up/Down
        g_InControlState.SLLeg.z = (ly - 128)/2; //Left Stick Up/Down

        // Hold single leg in place
        if (ps2x.ButtonPressed(PSB_R2)) { // R2 Button Test
          MSound(1, 50, 2000);  
          g_InControlState.fSLHold = !g_InControlState.fSLHold;
        }
      }
#endif
#ifdef OPT_GPPLAYER
      //[GPPlayer functions]
      if (ControlMode == GPPLAYERMODE) {

        // Lets try some speed control... Map all values if we have mapped some before
        // or start mapping if we exceed some minimum delta from center
        // Have to keep reminding myself that commander library already subtracted 128...
        if (g_ServoDriver.FIsGPSeqActive() ) {
          if ((g_sGPSMController != 32767)  
            || (ps2x.Analog(PSS_RY) > (128+16)) || (ps2x.Analog(PSS_RY) < (128-16)))
          {
            // We are in speed modify mode...
            short sNewGPSM = map(ps2x.Analog(PSS_RY), 0, 255, -200, 200);
            if (sNewGPSM != g_sGPSMController) {
              g_sGPSMController = sNewGPSM;
              g_ServoDriver.GPSetSpeedMultiplyer(g_sGPSMController);
            }

          }
        }

        //Switch between sequences
        if (ps2x.ButtonPressed(PSB_SELECT)) { // Select Button Test
          if (!g_ServoDriver.FIsGPSeqActive() ) {
            if (GPSeq < 5) {  //Max sequence
              MSound(1, 50, 1500);
              GPSeq = GPSeq+1;
            } 
            else {
              MSound(2, 50, 2000, 50, 2250);
              GPSeq=0;
            }
          }
        }
        //Start Sequence
        if (ps2x.ButtonPressed(PSB_R2))// R2 Button Test
          if (!g_ServoDriver.FIsGPSeqActive() ) {
          g_ServoDriver.GPStartSeq(GPSeq);
            g_sGPSMController = 32767;  // Say that we are not in Speed modify mode yet... valid ranges are 50-200 (both postive and negative... 
          }
          else {
            g_ServoDriver.GPStartSeq(0xff);    // tell the GP system to abort if possible...
            MSound (2, 50, 2000, 50, 2000);
          }


      }
#endif // OPT_GPPLAYER

      //Calculate walking time delay
      g_InControlState.InputTimeDelay = 128 - max(max(abs(lx - 128), abs(ly - 128)), abs(ps2x.Analog(PSS_RX) - 128));
    }

    //Calculate g_InControlState.BodyPos.y
    g_InControlState.BodyPos.y = min(max(g_BodyYOffset + g_BodyYShift,  0), MAX_BODY_Y);
    
#ifdef OPT_DYNAMIC_ADJUST_LEGS  
    if (sLegInitXZAdjust || sLegInitAngleAdjust) {
      // User asked for manual leg adjustment - only do when we have finished any previous adjustment

      if (!g_InControlState.ForceGaitStepCnt) {
        if (sLegInitXZAdjust)
          g_fDynamicLegXZLength = true;

        sLegInitXZAdjust += GetLegsXZLength();  // Add on current length to our adjustment...
        // Handle maybe change angles...
        if (sLegInitAngleAdjust) 
            RotateLegInitAngles(sLegInitAngleAdjust);
        // Give system time to process previous calls
        AdjustLegPositions(sLegInitXZAdjust);
      }
    }    
#endif
    
    if (fAdjustLegPositions)
      AdjustLegPositionsToBodyHeight();    // Put main workings into main program file
  } 
  else {
    // We may have lost the PS2... See what we can do to recover...
    if (g_sPS2ErrorCnt < MAXPS2ERRORCNT)
      g_sPS2ErrorCnt++;    // Increment the error count and if to many errors, turn off the robot.
    else if (g_InControlState.fRobotOn)
      PS2TurnRobotOff();
    ps2x.reconfig_gamepad();
  }
}

//==============================================================================
// PS2TurnRobotOff - code used couple of places so save a little room...
//==============================================================================
void PS2TurnRobotOff(void)
{
  //Turn off
  g_InControlState.BodyPos.x = 0;
  g_InControlState.BodyPos.y = 0;
  g_InControlState.BodyPos.z = 0;
  g_InControlState.BodyRot1.x = 0;
  g_InControlState.BodyRot1.y = 0;
  g_InControlState.BodyRot1.z = 0;
  g_InControlState.TravelLength.x = 0;
  g_InControlState.TravelLength.z = 0;
  g_InControlState.TravelLength.y = 0;
  g_BodyYOffset = 0;
  g_BodyYShift = 0;
#ifdef OPT_SINGLELEG
  g_InControlState.SelectedLeg = 255;
#endif  
  g_InControlState.fRobotOn = 0;
  AdjustLegPositionsToBodyHeight();    // Put main workings into main program file
}




