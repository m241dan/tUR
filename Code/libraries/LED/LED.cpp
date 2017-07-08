

#include "LED.h"

LED::LED(uint8_t ledPin){
	this->pin=ledPin;
	this->status=LOW;
	pinMode(this->pin,OUTPUT);
}

bool LED::getState(){ return status; }

void LED::on(void){
	digitalWrite(pin,HIGH);
	this->status=true;
}

void LED::off(void){
	digitalWrite(pin,LOW);
	this->status=false;
}
	
void LED::toggle(void){
	status ? off() : on();
}

void LED::blink(unsigned int time, byte times){
	for (byte i=0; i<times; i++){
		toggle();
		delay(time/2);
		toggle();
		delay(time/2);
	}
}

//assume PWM
void LED::setValue(byte val){
	analogWrite(pin,val);
	val<=127 ? this->status=false : this->status=true;
}

//assume PWM
void LED::fadeIn(unsigned int time){
	for (byte value = 0 ; value < 255; value+=5){ 
		analogWrite(pin, value);
		delay(time/(255/5));
	} 
	on();
}

//assume PWM
void LED::fadeOut(unsigned int time){
	for (byte value = 255; value >0; value-=5){ 
		analogWrite(pin, value); 
		delay(time/(255/5)); 
	}
	off();
}

extern LED DEBUG_LED(13);

