
//************* Sawtooth like PWM on Pin 5 **************************************//
//************* Prof. Mallebrein  HS-Esslingen 2024 ****************************//
// DC-Motor Control Pin pwm
int pwmPin = 5;  // 5 at Nano (high frequency)
int pwm = 0;  // output for PWM: could be also integer (0...255)

// Calculation Grid and Printing Speed variables
int           millisGrid = 100;   // gridtime in ms 
unsigned long oldMillis = 0;        // for grid generation

void setup() {
	// put your setup code here, to run once:
	pinMode(pwmPin, OUTPUT);   
}

void loop() {
	// put your main code here, to run repeatedly:

	if (millis()-oldMillis >= millisGrid)
	{
		pwm = pwm + 1;
		analogWrite(pwmPin, pwm);   // pwm controls motor speed (min 0 max. 255) 
		if (pwm > 253) pwm = 1;
		oldMillis = oldMillis + millisGrid; // counts up oldMillis
	}
}
