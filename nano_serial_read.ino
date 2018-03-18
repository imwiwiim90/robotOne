




void setup() {
		
        Serial.begin(9600);
}

int in_serial = 0;
void loop() {


        if (Serial.available() > 0) {
                in_serial = Serial.read();
                if (in_serial) {

                }
        }
}