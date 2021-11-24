// Cruise Control voor BBM
char MuchoFunosCC[35] = "### Mucho Funos CC V0.8, incl. GPS";

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <TMC2130Stepper.h>
#include <Arduino.h>
#include <SPI.h>
#include <U8g2lib.h>
  
#define POTMETER_BIG_PIN    A0
#define POTMETER_SMALL_PIN  A1
#define EN_PIN              46 
#define STEP_PIN            48 
#define DIR_PIN             49  
#define MISO_PIN            50
#define MOSI_PIN            51
#define SCK_PIN             52
#define CS_PIN              53
#define ENDSTOP_PIN         02
#define PLUS_PIN            19
#define STOP_PIN            07  // temp pin because temp not in use TODO: fase out, becaus we use the Arduino external Reset button
#define MINUS_PIN           18
#define FROM_ZERO           HIGH
#define TOWARDS_ZERO        LOW
#define STEPS_PER_MM        804,5977011494250                                     // MEASURED

// Global Constants
const int LoopSleep_ms             = 50;                                       // must be between 30 - 80 for GPS to work w/o DIAGs
const int  DisplayWaitLoops         = 500 / LoopSleep_ms;                     // every 500 msecs
const int   LeverMovedWaitLoops      = 200 / LoopSleep_ms;                   // every 200 msecs
const int    EffectCheckWaitLoops     = 1000 / LoopSleep_ms;                // every 1000 msecs
const int     BlinkWaitLoops           = 100 / LoopSleep_ms;               // every 100 msecs
const int      MaxWarningLoops          = 500 / LoopSleep_ms;             // display 500 msecs
const int       MaxTrottle_PositionDiff  = 5;                            // max diff degrees 0 - 360
const int        MaxCorrectionAttempts    = 15;                         // max CorrectionSteps within which speed must have shown some improvement
const int         CorrectionSteps          = 80;                       // nr of steps the stepper motor does per Correction
const int          MaxPlusMinSpeed          = 5;                      // max 5 kmh plus or minus, in 0.1 kmh
const int           MaxTotalSpeed            = 31;                   // max 30 (+1) kmh with CC
const int            MinTotalSpeed            = 4;                  // min 5 (-1) khm with CC

const int             DelayAfterCorrection_ms   = 10;
const int              Step2StepDelay_mus        = 100;           // tested, must be >= 40
const unsigned int      Zero2MiddlePosition_steps = 16000;       // 200 mm
const unsigned int       MaxTravel                 = 8000;      // 100 mm
const unsigned int        MinPosition               = Zero2MiddlePosition_steps - MaxTravel;
const unsigned int         MaxPosition               = Zero2MiddlePosition_steps + MaxTravel;
static const int                                RXPin = 10, TXPin = 11;  // not all pins on the Mega are suitable for SoftwareSerial RX! 10 is OK.
static const uint32_t                          GPSBaud = 9600;

// Global Vars
volatile bool               EndStopReached         = false; 
volatile bool                PlusPressed          = false; 
volatile bool                 StopPressed        = false; 
volatile bool                  MinusPressed     = false; 
byte                            Direction;

bool                       Blink;
bool                      ArrowOn;
long int                 CurrentPosition;
unsigned long           LastDisplay             = 0UL;
unsigned long          LastDiag                 = 0UL;
bool                  TooManyPlusMinusPresses   = false;
bool                 OutsideSpeedRange          = false;
int                 WaitinOnEffectCounter       = 0;
int                WaitOutsideRangeCounter      = 0;
int               WaitMaxDisplayCounter         = 0;
int              CurrentBlinkCounter            = 0;
int             CurrentDisplayCounter           = 0;
int            CurrentLeverMovedCounter         = 0;
int           EffectCheckNumber                 = 0;
int          SetSpeed                           = -1;
int         CurrentSpeed                        = -1;
int        StartSpeed                           = -1;
int       CCStartSpeed                          = -1;
int      CorrectionAttempts                     = 0;
char    DirectionNeeded                         = ' ';
long   ThrottleStartPosition                    = -1;
char  Max_str[5]                                = " ";  // empty or "max!"


// intiation statements that cannot reside within a function
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2( U8G2_R0 );  /* u8g2 Display Constructor */
TinyGPSPlus gps;                                    // The TinyGPS++ object
SoftwareSerial SwSerial(RXPin, TXPin);             // The serial connection to the GPS device


// the Program
void setup() 
{
  
  Serial.begin(9600);
  Serial.println( "" );
  Serial.print( MuchoFunosCC );
  Serial.print( " starting ...\n"  );

  u8g2.begin();

  SwSerial.begin(GPSBaud);
  
  TMC2130Stepper driver = TMC2130Stepper(EN_PIN, DIR_PIN, STEP_PIN, CS_PIN, MOSI_PIN, MISO_PIN, SCK_PIN);
  driver.begin();            // Initiate pins and registeries
  
  pinMode( ENDSTOP_PIN, INPUT_PULLUP );
  pinMode( PLUS_PIN,    INPUT_PULLUP );
  pinMode( STOP_PIN,    INPUT_PULLUP );
  pinMode( MINUS_PIN,   INPUT_PULLUP );

  attachInterrupt( digitalPinToInterrupt( ENDSTOP_PIN ), ISR_EndStopReached, FALLING );
  attachInterrupt( digitalPinToInterrupt( PLUS_PIN ),    ISR_PlusPressed,    FALLING );
  attachInterrupt( digitalPinToInterrupt( STOP_PIN ),    ISR_StopPressed,    FALLING );
  attachInterrupt( digitalPinToInterrupt( MINUS_PIN ),   ISR_MinusPressed,   FALLING );
  
} // setup

void loop() {
  
  ResetCC();
  
  while( true )
  {
    DisplayBeautifulThings();  // logging
    UpdateDisplay();

// TODO: LeverMoved with pot meter unreliable, goes off unintended
//    if( LeverMoved() ) 
//    {
//      Serial.println( "Levermoved!!");
//      break;          // stop/reset when lever moved
//    }
    if( StopPressed == true ) 
    {
      Serial.println( "StopPressed!" );
      break;         // stop/reset when STOP button pressed
    }
    if( PlusPressed == true || MinusPressed == true ) ProcessPlusMinusPress() ;

    if( SetSpeed > -1 && CurrentSpeed > -1)
    {
      
      if(    ( DirectionNeeded == ' ' && CurrentSpeed == SetSpeed )
          || ( DirectionNeeded == '+' && CurrentSpeed >= SetSpeed )
          || ( DirectionNeeded == '-' && CurrentSpeed <= SetSpeed )
        )  // no adjustment actions needed
      {  
        DirectionNeeded = ' ';  
        CorrectionAttempts = 0;
      }
      else
      {        
        if( CorrectionAttempts < MaxCorrectionAttempts && WaitinOnEffectCounter == 0 ) // if waiting on effect > 0, no extra corrections please
        {
          if( CurrentSpeed < SetSpeed )            // this is where it happens, the speed corrections  //
          {
            if(  CurrentPosition <= MinPosition )
            {
              Serial.println( "Min motorposition reached: " );
              Serial.println( CurrentPosition );
            }
            else
            {
              DirectionNeeded = '+';
              MoveSteps( TOWARDS_ZERO, CorrectionSteps );
              CorrectionAttempts++;  
            }
          } // if
          else if ( CurrentSpeed > SetSpeed)
          {
            if( CurrentPosition >= MaxPosition ) Serial.println( "Max motorposition reached" );
            else
            {
              DirectionNeeded = '-';
              MoveSteps( FROM_ZERO, CorrectionSteps );
              CorrectionAttempts++;  
            }
          } // else if
        
        } // if
        
        if( CorrectionEffect( ++EffectCheckNumber ) ) CorrectionAttempts = 0; 
          
      } // else 
              
    } // if 

//    ReadGPSandDelay( LoopSleep_ms ); 
    SimulateCurrentSpeed();  delay( LoopSleep_ms );  // for testing w/o GPS with potmeter speed
    
  } // while
} // Loop()


int CorrectionEffect( int CheckNrDummy )  // checks if the Corrections have _Resulted in smaller gap between SetSpeed and CurrentSpeed
{
  int _Result = 0;
 
  if( EffectCheckNumber == 1 )  // first check, just start the waittime
  {
    WaitinOnEffectCounter = 1;
    StartSpeed   = CurrentSpeed;
    return 0;
  }

  if( WaitinOnEffectCounter++ < EffectCheckWaitLoops ) return 0; // wait some more
  WaitinOnEffectCounter = 0;

  if( DirectionNeeded == '+' )
  {
    if( CurrentSpeed > StartSpeed ) _Result = 1;
    else                                  _Result = 0; 
  }
  if( DirectionNeeded == '-' )
  {
    if( CurrentSpeed < StartSpeed ) _Result = 1;
    else                                  _Result = 0; 
  }

  if( _Result == 1 )  EffectCheckNumber = 0;  // checking ready, reset number for next time

  return _Result;
  
} // CorrectionEffect( int FirstTime )


void DisplayBeautifulThings() 
{
    
  // Sends to display: 
  //    Current Speed  ( "--" if nog GPS reception) (blinking when MaxCorrectionAttempts is reached)
  //    Set Speed      ( "--" if no speed set yet)
  
  int _Result = 0;

  if( CurrentDisplayCounter++ < DisplayWaitLoops ) return ; // wait some more
  
  CurrentDisplayCounter = 0;

  if( CorrectionAttempts < MaxCorrectionAttempts )  Serial.print( "SOG: ");
  else                                              Serial.print( "!SOG!: ");
  if( CurrentSpeed > -1 ) Serial.print( CurrentSpeed  );
  else                       Serial.print( "--" );

  Serial.print( ", SetSpeed: ");
  if( SetSpeed > -1 ) Serial.print( SetSpeed  );
  else                Serial.print( "--" );  

  Serial.print(F(" SPEED      Fix Age="));
  Serial.print(gps.speed.age());
  Serial.println( " ") ;
 

}  // DisplayBeautifulThings()


void SimulateCurrentSpeed() // gets current speed SOG
{
  int _SOG;
  int _Result=0;

  //
  // TODO read out _SOG from GPS
  // fill _Result
  //
  // TEMP TEST CODE POTENTIOMETER
      _SOG = analogRead( POTMETER_BIG_PIN );
      _SOG = map( _SOG, 0, 1023, 10, 30 );
      _Result = 1;
  // TEMP TEST CODE
  
  if ( _Result == 1) CurrentSpeed  = _SOG;
  else               CurrentSpeed  = -1;

 
} // SimulateCurrentSpeed()


void ISR_EndStopReached()
{
  EndStopReached = true;
}
void ISR_MinusPressed()
{
  MinusPressed = true;
}
void ISR_PlusPressed()
{
  PlusPressed = true;
}
void ISR_StopPressed()
{
  StopPressed = true;
}

 
int LeverMoved() // dectection of _Movement of throttle lever
{
  long            _CurrentThrottlePosition=-1;
  long            _PositionDiff=0;
  unsigned int    _Movement=0;

  if( CurrentLeverMovedCounter++ < LeverMovedWaitLoops ) return 0; // wait some more
  
  if( ThrottleStartPosition == -1 ) return 0;  // CC not activated yet

  _CurrentThrottlePosition = ReadThrottlePosition();

  _PositionDiff = _CurrentThrottlePosition - ThrottleStartPosition;
  _Movement = abs( _PositionDiff );
  
  if ( _Movement > MaxTrottle_PositionDiff )  return 1;
  else                                        return 0;
   
} // LeverMoved()


void MoveSteps( byte _Direction, long int _Steps )
{
  long int _StepCounter;

  digitalWrite( DIR_PIN, _Direction );
  for( _StepCounter = 0; _StepCounter < _Steps; _StepCounter++ )
  {
    digitalWrite( STEP_PIN, HIGH );
    delayMicroseconds( Step2StepDelay_mus ); 
    digitalWrite( STEP_PIN, LOW );
    delayMicroseconds( Step2StepDelay_mus );
  }

  if( _Direction == TOWARDS_ZERO )  CurrentPosition -= _Steps;  
  else                              CurrentPosition += _Steps; 

} // MoveSteps()


int ProcessPlusMinusPress() 
{
  if ( SetSpeed == -1 )  // first plus/minus press, CC must be activated
  {
    if(( PlusPressed == true || MinusPressed == true ) && ( CurrentSpeed < MinTotalSpeed || CurrentSpeed > MaxTotalSpeed ))
    {
      OutsideSpeedRange = true;
      PlusPressed  = false; 
      MinusPressed = false; 
      return;
    } // if
    else OutsideSpeedRange = false;
  
    SetSpeed     = CurrentSpeed;
    CCStartSpeed = CurrentSpeed;
    ThrottleStartPosition = ReadThrottlePosition(); 

    PlusPressed  = false; 
    MinusPressed = false; 
    return;
  }

  TooManyPlusMinusPresses = false;

  if( PlusPressed == true )
  { 
    if( SetSpeed >= CCStartSpeed + MaxPlusMinSpeed || SetSpeed >= MaxTotalSpeed )  
    {
      TooManyPlusMinusPresses = true;
      WaitMaxDisplayCounter = 0;
    }
    else                                                                                          SetSpeed++;
    PlusPressed  = false; 
  }

  if( MinusPressed == true )                             
  {
    if( SetSpeed <= CCStartSpeed - MaxPlusMinSpeed || SetSpeed <= MinTotalSpeed )
    {
      TooManyPlusMinusPresses = true;
      WaitMaxDisplayCounter = 0;
    }
    else                                                                                          SetSpeed--;
    MinusPressed = false; 
  }

} // ProcessPlusMinusPress()


void ReadGPSandDelay( unsigned long _delay_ms )
{
  //TODO: extract only $GPVTG + maybe ditch TinyGPS+?
  
  unsigned long _start_ms = millis();

  do 
  {
    while (SwSerial.available()) gps.encode(SwSerial.read());
  
    if (gps.speed.isUpdated())
    {
      CurrentSpeed = gps.speed.kmph();
    }
    else if ( gps.speed.age() > 2000 )  // TODO tunen
    { 
      // TODO warning on Display
    
      if( millis() - LastDiag > 5000 )
      {
      
         Serial.print(F( "--DIAGS-- CharsProcessed=         " ));
         Serial.print(     gps.charsProcessed() );
         Serial.print(F( " Sentences-with-Fix=" ));
         Serial.print(     gps.sentencesWithFix() );
         Serial.print(F( " Failed-checksum=" ));
         Serial.print(     gps.failedChecksum() );
         Serial.print(F( " Passed-checksum=" ));
         Serial.print(     gps.passedChecksum() );
         Serial.print(F( " SPEED Fix Age=" ));
         Serial.println(   gps.speed.age() );      // TODO Speed Age == 4294967295, means no Speed acquired yet since startup?

         if (gps.charsProcessed() < 10)  Serial.println(F("--WARNING: No GPS data.     Check wiring.--"));

         LastDiag = millis();
       
      } // if
        
    } // else if 

  } while ( millis() - _start_ms < _delay_ms );

} // ReadGPSandDelay


long ReadThrottlePosition()
{
  long  _CurrentThrottlePosition=-1;
   int  _Result=0;

  //
  //TODO get throttle pos. into _CurrentThrottlePosition
  //

  // TEMP TEST CODE POTENTIOMETER
      _CurrentThrottlePosition = analogRead( POTMETER_SMALL_PIN );
      _CurrentThrottlePosition = map( _CurrentThrottlePosition, 0, 1023, 10, 30 );
      _Result = 1;
  // TEMP TEST CODE
  
  if ( _Result == 1) return _CurrentThrottlePosition;
  else               return -1;
  
} // ReadThrottlePosition()


void Reset2MiddlePosition()
{
  long int _StepCounter = 0;

  // go to position 0
  digitalWrite( DIR_PIN, TOWARDS_ZERO );
  EndStopReached = false;
  for( _StepCounter = 0; EndStopReached == false; _StepCounter++ )
  {
    digitalWrite( STEP_PIN, HIGH );
    delayMicroseconds( Step2StepDelay_mus ); 
    digitalWrite( STEP_PIN, LOW );
    delayMicroseconds( Step2StepDelay_mus );
  }
  delay( 10 );

  EndStopReached = false;
  CurrentPosition = 0;
  Serial.print( "motorposition reached: " );
  Serial.println( CurrentPosition );

  // go from position 0 to middle position
  MoveSteps( FROM_ZERO, Zero2MiddlePosition_steps );

  Serial.print( "motorposition reached: " );
  Serial.println( CurrentPosition );
  
  delay( DelayAfterCorrection_ms );
} // Reset2MiddlePosition()


void ResetCC() 
{
  // reset global vars and reset Actuator to middle position
  Serial.print( "-- Resetting --\n" );
  
  SetSpeed = -1;
  CurrentSpeed = -1;
  StartSpeed = -1;
  CCStartSpeed = -1;
  CorrectionAttempts = 0;
  DirectionNeeded = ' ';
  ThrottleStartPosition = -1;
  EffectCheckNumber = 0;
  EndStopReached  = false;
  PlusPressed    = false; 
  StopPressed   = false; 
  MinusPressed = false; 

  UpdateDisplay();

  Reset2MiddlePosition();

} // ResetCC()


void UpdateDisplay( void )
{
  char _SOG_str[3];          // khm 0 - 99
  char _SetSpeed_str[3];    // kmh 4 - 30
  char _Dir_str[2];        //  arrow down or arrow up (one char)
  
  char OutsideRange_str[18] = "range = 5 - 30!"; 
  //TODO Max / Min vars verwerken in OutsideRange_str
  // MaxTotalSpeed         = 30;                  // max 30 kmh with CC
  // MinTotalSpeed         = 4;                 // min 4 khm with CC

  if( CurrentSpeed == -1 || CurrentSpeed > 99 )   sprintf( _SOG_str, "--" );
  else                                          sprintf( _SOG_str, "%2d", CurrentSpeed );
  if( SetSpeed == -1 )                          sprintf( _SetSpeed_str, "--" );
  else                                            sprintf( _SetSpeed_str, "%2d", SetSpeed );
 
  switch( DirectionNeeded )
  {
    case '+':
                 sprintf( _Dir_str, "\x43" );   // \x43 = arrow up in u8g2_font_open_iconic_arrow_2x_t
          break;
    case '-':
               sprintf( _Dir_str, "\x40" );  //  \x40 = arrow down in u8g2_font_open_iconic_arrow_2x_t
        break;
    case ' ':
             sprintf( _Dir_str, " " ); 
      break;
  } // switch

  if( CorrectionAttempts < MaxCorrectionAttempts )
  {
    Blink = false; 
    ArrowOn = true;
  }
  else                                             
  {
     if( Blink == false )
     {
       CurrentBlinkCounter = 0;  // starting the blink
       ArrowOn = true;          // start with arrow on
     }
     Blink = true;
 
     if( CurrentBlinkCounter++ > BlinkWaitLoops ) 
     {
       ArrowOn = !ArrowOn;
       CurrentBlinkCounter = 0;
     }
  } // else

  if( ArrowOn == false ) sprintf( _Dir_str, " " );

  if( TooManyPlusMinusPresses == true && WaitMaxDisplayCounter == 0 )
  {
    sprintf( Max_str, "max!" );  // first loop 
    WaitMaxDisplayCounter++;
    TooManyPlusMinusPresses = false;
  }
  if( WaitMaxDisplayCounter > 0 )
  {
    if( WaitMaxDisplayCounter++ > MaxWarningLoops )
    {
      WaitMaxDisplayCounter = 0;
      sprintf( Max_str, "    " );  
    }
  } // if

  if( OutsideSpeedRange == true && WaitOutsideRangeCounter == 0 )  WaitOutsideRangeCounter++;
  if( WaitOutsideRangeCounter > 0 )
  {
    if( WaitOutsideRangeCounter++ > MaxWarningLoops )
    {
      OutsideSpeedRange = false;
      WaitOutsideRangeCounter = 0;
    }
  }
  
  u8g2.clearBuffer();
    u8g2.setFont( u8g2_font_fur11_tr);
  if( OutsideSpeedRange == true )
  {
    u8g2.drawStr( 9,17, OutsideRange_str );
  }
  else
  {
    u8g2.drawStr( 8,17, Max_str );
    u8g2.drawStr( 55,17, "set");
    u8g2.setFont( u8g2_font_inr24_mf);
    u8g2.drawStr( 88,25, _SetSpeed_str );
  }
    u8g2.setFont( u8g2_font_fur11_tr);
    u8g2.drawStr( 52,54, "sog");
    u8g2.setFont( u8g2_font_inr24_mf);
    u8g2.drawStr( 88,62, _SOG_str );
    u8g2.setFont( u8g2_font_open_iconic_arrow_2x_t);
    u8g2.drawStr( 15,59, _Dir_str );
  u8g2.sendBuffer();
 
} // UpdateDisplay()

// EOF
