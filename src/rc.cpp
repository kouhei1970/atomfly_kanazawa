#include <Ps3Controller.h>
#include "rc.hpp"

int player = 0;
int battery = 0;

//RC
volatile float Stick[16];


void notify()
{
    //--- Digital cross/square/triangle/circle button events ---
    if( Ps3.event.button_down.circle )
    {
        //Serial.println("Started pressing the circle button");
        Stick[LOG] = 1;
    }
    if( Ps3.event.button_up.circle )
    {
        //Serial.println("Released the circle button");
    }
    if( Ps3.event.button_down.cross )
    {
        //Serial.println("Started pressing the cross button");
        Stick[LOG] = 0;
    }
    if( Ps3.event.button_up.cross )
    {
        //Serial.println("Released the cross button");
    }

    if( Ps3.event.button_down.up )
    {
        Stick[DPAD_UP]=1;
    }
    if( Ps3.event.button_up.up ){
        Stick[DPAD_UP]=0;
    }
    if( Ps3.event.button_down.down )
    {
        Stick[DPAD_DOWN]=1;
    }
    if( Ps3.event.button_up.down ){
        Stick[DPAD_DOWN]=0;
    }
    if( Ps3.event.button_down.left )
    {
        Stick[DPAD_LEFT]=1;
    }
    if( Ps3.event.button_up.left ){
        Stick[DPAD_LEFT]=0;
    }
    if( Ps3.event.button_down.right )
    {
        Stick[DPAD_RIGHT]=1;
    }
    if( Ps3.event.button_up.right ){
        Stick[DPAD_RIGHT]=0;
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
    Stick[THROTTLE] = (float)Ps3.data.analog.stick.ry/THROTTLE_MIN;
    if (Stick[THROTTLE] < 0) Stick[THROTTLE] = 0;
    //Rudder
    Stick[RUDDER] =  Ps3.data.analog.stick.rx;
    if (Stick[RUDDER]>0) Stick[RUDDER] =  Stick[RUDDER]/RUDDER_MAX;
    else if (Stick[RUDDER]<0) Stick[RUDDER] = -Stick[RUDDER]/RUDDER_MIN;
    Stick[RUDDER] = -Stick[RUDDER];
    //Elevator
    Stick[ELEVATOR] =  Ps3.data.analog.stick.ly;
    if (Stick[ELEVATOR]>0) Stick[ELEVATOR] =  Stick[ELEVATOR]/ELEVATOR_MAX;
    else if (Stick[ELEVATOR]<0) Stick[ELEVATOR] = -Stick[ELEVATOR]/ELEVATOR_MIN;
    //Aileron
    Stick[AILERON]  =   Ps3.data.analog.stick.lx;
    if (Stick[AILERON]>0) Stick[AILERON] = Stick[AILERON]/AILERON_MAX;
    else if (Stick[AILERON]<0) Stick[AILERON] = - Stick[AILERON]/AILERON_MIN;

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

bool rc_isconnected(void)
{
    return Ps3.isConnected();
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
