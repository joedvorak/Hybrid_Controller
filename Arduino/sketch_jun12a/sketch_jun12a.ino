char buffer[64];
char *ptr;
//long rpmArray[29];
Canbus.ecu_req(ENGINE_RPM,buffer);
double data;
data = strtod(buffer, &ptr);


void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Extracting Engine RPM");
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
