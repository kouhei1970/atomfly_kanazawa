#include <Ps3Controller.h>
#include "rc.hpp"

int player = 0;
int battery = 0;

//RC
volatile float Stick_throttle;
volatile float Stick_roll;
volatile float Stick_pitch;
volatile float Stick_yaw;
volatile uint8_t Log_sw;


void notify()
{
    //--- Digital cross/square/triangle/circle button events ---
    if( Ps3.event.button_down.circle )
    {
        //Serial.println("Started pressing the circle button");
        Log_sw = 1;
    }
    if( Ps3.event.button_up.circle )
    {
        //Serial.println("Released the circle button");
    }
    if( Ps3.event.button_down.cross )
    {
        //Serial.println("Started pressing the cross button");
        Log_sw = 0;
    }
    if( Ps3.event.button_up.cross )
    {
        //Serial.println("Released the cross button");
    }

    //---------- Digital select/start/ps button events ---------
    if( Ps3.event.button_down.select )
        Serial.println("Started pressing the select button");
    if( Ps3.event.button_up.select )
        Serial.println("Released the select button");

    if( Ps3.event.button_down.start )
        Serial.println("Started pressing the start button");
    if( Ps3.event.button_up.start )
        Serial.println("Released the start button");

    if( Ps3.event.button_down.ps )
        Serial.println("Started pressing the Playstation button");
    if( Ps3.event.button_up.ps )
        Serial.println("Released the Playstation button");


    //---------------- Analog stick value events ---------------
   if( abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 2 ){
       //Serial.print("Moved the left stick:");
       //Serial.print(" x="); Serial.print(Ps3.data.analog.stick.lx, DEC);
       //Serial.print(" y="); Serial.print(Ps3.data.analog.stick.ly, DEC);
       //Serial.println();

    }

   if( abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2 ){
       //Serial.print("Moved the right stick:");
       //Serial.print(" x="); Serial.print(Ps3.data.analog.stick.rx, DEC);
       //Serial.print(" y="); Serial.print(Ps3.data.analog.stick.ry, DEC);
       //Serial.println();
   }

   //---------------------- Battery events ---------------------
    if( battery != Ps3.data.status.battery ){
        battery = Ps3.data.status.battery;
        Serial.print("The controller battery is ");
        if( battery == ps3_status_battery_charging )      Serial.println("charging");
        else if( battery == ps3_status_battery_full )     Serial.println("FULL");
        else if( battery == ps3_status_battery_high )     Serial.println("HIGH");
        else if( battery == ps3_status_battery_low)       Serial.println("LOW");
        else if( battery == ps3_status_battery_dying )    Serial.println("DYING");
        else if( battery == ps3_status_battery_shutdown ) Serial.println("SHUTDOWN");
        else Serial.println("UNDEFINED");
    }

    //Throttle
    Stick_throttle = (float)Ps3.data.analog.stick.ry/THROTTLE_MIN;
    if (Stick_throttle < 0) Stick_throttle = 0;
    //Rudder
    Stick_yaw =  Ps3.data.analog.stick.rx;
    if (Stick_yaw>0) Stick_yaw =  Stick_yaw/RUDDER_MAX;
    else if (Stick_yaw<0) Stick_yaw = -Stick_yaw/RUDDER_MIN;
    Stick_yaw = -Stick_yaw;
    //Elevator
    Stick_pitch =  Ps3.data.analog.stick.ly;
    if (Stick_pitch>0) Stick_pitch =  Stick_pitch/ELEVATOR_MAX;
    else if (Stick_pitch<0) Stick_pitch = -Stick_pitch/ELEVATOR_MIN;
    //Aileron
    Stick_roll  =   Ps3.data.analog.stick.lx;
    if (Stick_roll>0) Stick_roll = Stick_roll/RUDDER_MAX;
    else if (Stick_roll<0) Stick_roll = - Stick_roll/RUDDER_MIN;

    //Serial.printf("%6.3f %6.3f %6.3f %6.3f %4d\n",Stick_throttle, Stick_roll, Stick_pitch, Stick_yaw, Log_sw);

}

void onConnect(){
    Serial.println("Connected.");
}

void rc_init(void)
{
    Ps3.attach(notify);
    Ps3.attachOnConnect(onConnect);
    Ps3.begin(BTID);
    Serial.println("Ready.");
}

void rc_end(void)
{
    Ps3.end();
}

void rc_demo()
{
    if(!Ps3.isConnected())
        return;

    //-------------------- Player LEDs -------------------
    Serial.print("Setting LEDs to player "); Serial.println(player, DEC);
    Ps3.setPlayer(player);

    player += 1;
    if(player > 10) player = 0;

    delay(2000);
}
